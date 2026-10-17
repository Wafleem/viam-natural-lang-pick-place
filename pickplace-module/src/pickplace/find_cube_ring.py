"""
Detect the RoboMaster ore cube and the black ring as separate objects.

  cube: white with black markings (corner brackets, solid squares, R logo, barcode).
        It is found as a group of black corner markers (squares, L brackets) close together.
  ring: a round dark shape with one big hole in the middle (the table shows through).
"""

from itertools import combinations

import cv2
import numpy as np

WORK_WIDTH = 800        # frames are resized to this width so the thresholds below hold at any resolution

DARK_V_MAX = 100        # "black" = brightness (HSV V) below this. Raise if markings are missed in dim light.
RING_MIN_AREA = 2000    # px at WORK_WIDTH
RING_THIN_PX = 13       # dark lines thinner than this (cables, tape) are erased before looking for rings
RING_HOLE_MIN = 0.10    # biggest hole area / outer area: a ring has one real hole, a dark patch does not
RING_HOLE_MAX = 0.70
RING_MIN_SOLIDITY = 0.90  # ring outline is convex; cable loops and box edges are not
RING_MIN_ELLIPSE_FIT = 0.85  # outline area / fitted-ellipse area: the ring is round (or an ellipse when tilted)
RING_MAX_HOLE_OFFSET = 0.10  # the hole sits in the middle (offset as a fraction of the ring's size)
MARK_MIN_AREA = 30      # smallest marking (one barcode stripe)
MARK_MAX_AREA = 8000    # larger dark blobs are not markings (shadows, clothing, ...)
MARK_MAX_ASPECT = 2.5   # long thin dark shapes (table edges, tape lines) are not markings
CLUSTER_GAP = 80        # markings closer than this many px belong to the same cube (spans the gap between two faces)
CLUSTER_SIZE_MULT = 3.0 # ...or closer than this many times the marking's size (corner squares sit far apart)
RING_CLEARANCE = 25     # dark bits this close to a ring (tape holding it down) are not cube markings
CUBE_MIN_MARKS = 4      # a cube shows at least this many markings
CUBE_MIN_CORNERS = 2    # ...including at least this many corner markers (solid squares or L brackets)
CORNER_MIN_SIZE = 20    # px at WORK_WIDTH; corner markers are bigger than letters or barcode stripes
CORNER_SIZE_RATIO = 0.6 # corner markers smaller than this fraction of the typical one are ignored


def find_objects(bgr: np.ndarray):
    scale = WORK_WIDTH / bgr.shape[1]
    small = cv2.resize(bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(cv2.GaussianBlur(small, (5, 5), 0), cv2.COLOR_BGR2HSV)
    dark = cv2.inRange(hsv, (0, 0, 0), (179, 255, DARK_V_MAX))
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    # Rings: erase thin dark lines first, so a cable lying across the ring doesn't spoil its shape.
    thick = cv2.morphologyEx(dark, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (RING_THIN_PX, RING_THIN_PX)))
    contours, hierarchy = cv2.findContours(thick, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    rings = []
    if hierarchy is not None:
        for i, c in enumerate(contours):
            if hierarchy[0][i][3] != -1:  # this contour is a hole; it's handled with its parent
                continue
            area = cv2.contourArea(c)
            holes = [contours[j] for j in range(len(contours)) if hierarchy[0][j][3] == i]
            if area >= RING_MIN_AREA and holes and len(c) >= 5 and _is_ring(c, max(holes, key=cv2.contourArea), area):
                (cx, cy), axes, angle = cv2.fitEllipse(c)
                rings.append({"center": (cx, cy), "bbox": cv2.boundingRect(c), "ellipse": ((cx, cy), axes, angle)})

    # Cube markings: every mid-sized dark blob.
    contours, hierarchy = cv2.findContours(dark, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    marks = []
    if hierarchy is not None:
        for i, c in enumerate(contours):
            if hierarchy[0][i][3] != -1:
                continue
            area = cv2.contourArea(c)
            if MARK_MIN_AREA <= area <= MARK_MAX_AREA:
                x, y, w, h = cv2.boundingRect(c)
                at_edge = x <= 1 or y <= 1 or x + w >= small.shape[1] - 1 or y + h >= small.shape[0] - 1
                if max(w, h) / max(1, min(w, h)) <= MARK_MAX_ASPECT and not at_edge:  # edge blobs: cables, clutter
                    marks.append((x, y, w, h, _is_corner_marker(c, area), c))

    # Drop dark bits stuck to a ring. Then group the corner markers (squares, L brackets) that sit
    # close together; each group of at least CUBE_MIN_CORNERS is a cube. Other markings (logo,
    # barcode) only count if they lie inside a group's area, so clutter can't chain onto the cube.
    marks = [m for m in marks if all(_gap(m[:4], r["bbox"]) > RING_CLEARANCE for r in rings)]
    groups = []
    for box in [m for m in marks if m[4]]:
        touching = [g for g in groups if any(_close(box[:4], b[:4]) for b in g)]
        merged = [box] + [b for g in touching for b in g]
        groups = [g for g in groups if g not in touching] + [merged]
    cubes = []
    for g in groups:
        # corner markers much smaller than the others are tape or specks next to the cube, not its corners
        typical = np.median([max(b[2], b[3]) for b in g])
        g = [b for b in g if max(b[2], b[3]) >= CORNER_SIZE_RATIO * typical]
        if len(g) < CUBE_MIN_CORNERS:
            continue
        x, y, w, h = _union(g)
        pad = max(max(b[2], b[3]) for b in g) // 2
        inside = [m for m in marks if not m[4] and m[0] >= x - pad and m[1] >= y - pad
                  and m[0] + m[2] <= x + w + pad and m[1] + m[3] <= y + h + pad]
        if len(g) + len(inside) < CUBE_MIN_MARKS:
            continue
        # Tight border: the smallest rotated rectangle around the corner markers' outlines. That only
        # fits when one face is seen from above (a marker sits in each corner of the rectangle); for a
        # side view of two faces, fall back to an upright box around all the markings.
        rect = _fit_face(g)
        if rect is not None:
            box = cv2.boxPoints(rect)
            (cx, cy), _, angle = rect
            cubes.append({"center": (cx, cy), "bbox": cv2.boundingRect(box.astype(np.int32)), "box": box,
                          "angle": (angle + 45) % 90 - 45, "markings": len(g) + len(inside)})
        else:
            x, y, w, h = _union(g + inside)
            box = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]], np.float32)
            cubes.append({"center": (x + w / 2, y + h / 2), "bbox": (x, y, w, h), "box": box,
                          "angle": None, "markings": len(g) + len(inside)})

    # Scale results back to the original image size.
    def up(o):
        x, y, w, h = o["bbox"]
        o["bbox"] = tuple(int(v / scale) for v in (x, y, w, h))
        o["center"] = (int(o["center"][0] / scale), int(o["center"][1] / scale))
        if "box" in o:
            o["box"] = (o["box"] / scale).astype(np.int32)
        if "ellipse" in o:
            (cx, cy), (a, b), ang = o["ellipse"]
            o["ellipse"] = ((cx / scale, cy / scale), (a / scale, b / scale), ang)
        return o

    return [up(c) for c in cubes], [up(r) for r in rings]


def _is_corner_marker(c, area):
    """Solid square (fills most of its box, 4 corners) or L bracket (fills about half, ~6 corners).
    Checked against both the upright box and the tightest rotated box, so it works whether the cube
    is turned on the table or seen at an angle."""
    corners = len(cv2.approxPolyDP(c, 0.04 * cv2.arcLength(c, True), True))
    _, _, bw, bh = cv2.boundingRect(c)
    _, (rw, rh), _ = cv2.minAreaRect(c)
    for w, h in ((bw, bh), (rw, rh)):
        if min(w, h) < CORNER_MIN_SIZE or max(w, h) / min(w, h) > 2.0:  # up to 2.0: seen at an angle
            continue
        fill = area / (w * h)
        if (fill >= 0.65 and corners == 4) or (0.25 <= fill < 0.65 and 5 <= corners <= 7):
            return True
    return False


def _is_ring(outer, hole, area):
    """A ring is a round, convex dark shape with one big hole in its middle."""
    hole_area = cv2.contourArea(hole)
    if not RING_HOLE_MIN <= hole_area / area <= RING_HOLE_MAX:
        return False
    if area / cv2.contourArea(cv2.convexHull(outer)) < RING_MIN_SOLIDITY:
        return False
    (cx, cy), (a, b), _ = cv2.fitEllipse(outer)
    if area / (np.pi * a * b / 4) < RING_MIN_ELLIPSE_FIT:
        return False
    m = cv2.moments(hole)
    hx, hy = m["m10"] / m["m00"], m["m01"] / m["m00"]
    return np.hypot(hx - cx, hy - cy) / max(a, b) <= RING_MAX_HOLE_OFFSET


def _fit_face(corner_marks):
    """Smallest rotated rectangle around the corner markers, if a marker sits in each of its four
    corners (one cube face seen from above). Retries without one or two markers, in case a dark bit
    on the cube's side was taken for a corner. None if nothing fits."""
    for drop in range(3):
        best = None
        for keep in combinations(corner_marks, len(corner_marks) - drop):
            if len(keep) < 4:
                break
            pts = np.vstack([m[5] for m in keep]).reshape(-1, 2).astype(np.float32)
            rect = cv2.minAreaRect(pts)
            reach = 0.2 * min(rect[1])
            if all(np.min(np.hypot(*(pts - corner).T)) <= reach for corner in cv2.boxPoints(rect)):
                if best is None or rect[1][0] * rect[1][1] > best[1][0] * best[1][1]:
                    best = rect
        if best is not None:
            return best
    return None


def _union(boxes):
    """Bounding box around a list of (x, y, w, h, ...) boxes."""
    x0 = min(b[0] for b in boxes)
    y0 = min(b[1] for b in boxes)
    x1 = max(b[0] + b[2] for b in boxes)
    y1 = max(b[1] + b[3] for b in boxes)
    return x0, y0, x1 - x0, y1 - y0


def _close(a, b):
    """True if two markings are near enough to belong to the same cube."""
    size = max(a[2], a[3], b[2], b[3])
    return _gap(a, b) < max(CLUSTER_GAP, CLUSTER_SIZE_MULT * size)


def _gap(a, b):
    """Distance between two boxes (0 if they overlap)."""
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    dx = max(0, max(ax, bx) - min(ax + aw, bx + bw))
    dy = max(0, max(ay, by) - min(ay + ah, by + bh))
    return (dx * dx + dy * dy) ** 0.5


def annotate(bgr, cubes, rings):
    out = bgr.copy()
    t = max(2, bgr.shape[1] // 400)
    for c in cubes:
        cv2.polylines(out, [c["box"]], True, (0, 200, 0), t)
        top = tuple(c["box"][np.argmin(c["box"][:, 1])])
        name = "cube" if c["angle"] is None else f'cube {c["angle"]:+.0f} deg'
        cv2.putText(out, name, (top[0], max(20, top[1] - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, t * 0.5, (0, 200, 0), t)
        _draw_center(out, c["center"], (0, 200, 0), t)
    for r in rings:
        cv2.ellipse(out, r["ellipse"], (255, 0, 255), t)
        x, y, w, h = r["bbox"]
        cv2.putText(out, "ring", (x, max(20, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, t * 0.5, (255, 0, 255), t)
        _draw_center(out, r["center"], (255, 0, 255), t)
    return out


def _draw_center(img, center, color, t):
    """Bullseye on the object's center, labeled with its pixel coordinates."""
    u, v = center
    r = 6 * t
    cv2.circle(img, (u, v), r, (255, 255, 255), t + 2)
    cv2.circle(img, (u, v), r, color, t)
    cv2.drawMarker(img, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 3 * r, t)
    text = f"({u}, {v})"
    org = (u + r + 4, v - r)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, t * 0.3, (255, 255, 255), t + 2, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, t * 0.3, (0, 0, 255), t, cv2.LINE_AA)
