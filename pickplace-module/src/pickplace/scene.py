"""Dynamic scene (branch dynamic-scene): re-locate boxes that may have been moved.

The cube and ring are already re-detected at the overview on every cycle. Boxes on `main`
are fixed config entries. Here, a box with `"track": true` (or every box when
`box_tracking: "live"`) is re-measured in the same overview frames used for the cube and
its POSE is updated. Its accepted opening, floor height and rim height are kept, so a
noisy view cannot shrink or deepen the cavity.

Scope and limits (scaffold):
- The box must be fully visible from the calibrated overview, with its orange Post-its.
- A measured box is matched to the configured box nearest to it (within MATCH_RADIUS_MM).
- Several frames must agree (TRACK_AGREE_MM / TRACK_AGREE_DEG) before a pose is replaced.
- Not done yet: views other than the overview, re-checking the goal right before release,
  and boxes the camera cannot see (they keep their configured pose and are reported).
"""
import copy
import math

import numpy as np

from .box import placement_xy
from .box_survey import candidate_entry, measure_box

MATCH_RADIUS_MM = 300.0
TRACK_AGREE_MM = 4.0
TRACK_AGREE_DEG = 1.5


def tracked_names(settings):
    if settings.box_tracking == 'live':
        return list(settings.boxes)
    return [n for n, b in settings.boxes.items() if b.get('track')]


def _wrap90(deg):
    return (deg+45) % 90-45


def measure_frames(frames, observations, settings):
    """[(bgr, depth, props, pose)] + matching observations -> per-frame box candidates."""
    found = []
    for (bgr, depth, props, pose), obs in zip(frames, observations):
        table_z = obs.get('observed_table_z_mm')
        try:
            raw = measure_box(bgr, depth, props, pose)
        except ValueError as e:
            found.append({'error': str(e)})
            continue
        found.append(candidate_entry('measured', raw, settings.calibration, table_z, settings.cube_size_mm))
    return found


def relocate(box, measurements):
    """New pose for one configured box from agreeing measurements, or None with a reason."""
    usable = [m for m in measurements if 'error' not in m]
    near = [m for m in usable if np.linalg.norm(np.subtract(m['floor_center_world_mm'][:2], box['floor_center_world_mm'][:2])) < MATCH_RADIUS_MM]
    if len(near) < 3:
        reasons = sorted({m['error'] for m in measurements if 'error' in m})
        return None, f'seen in {len(near)} frame(s); need 3' + (' ('+'; '.join(reasons)+')' if reasons else '')
    xy = np.array([m['floor_center_world_mm'][:2] for m in near])
    # Square-ish openings: compare yaw modulo 90 deg relative to the configured yaw.
    yaw = np.array([box['world_yaw_deg']+_wrap90(m['world_yaw_deg']-box['world_yaw_deg']) for m in near])
    if np.ptp(xy, axis=0).max() > TRACK_AGREE_MM or np.ptp(yaw) > TRACK_AGREE_DEG:
        return None, 'measurements disagree; keep the box still'
    new_xy, new_yaw = np.median(xy, axis=0), float(np.median(yaw))
    moved = copy.deepcopy(box)
    # Rigid move of the placement point with the box (it may be offset from the floor centre).
    a = math.radians(new_yaw-box['world_yaw_deg'])
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    offset = placement_xy(box)-np.asarray(box['floor_center_world_mm'][:2])
    moved['floor_center_world_mm'] = [*new_xy.tolist(), box['floor_center_world_mm'][2]]
    moved['placement_center_xy_mm'] = (new_xy+R @ offset).tolist()
    moved['world_yaw_deg'] = new_yaw
    return moved, None


def track_boxes(settings, frames, observations):
    """Settings copy with tracked boxes moved, plus a report. Unseen boxes keep their pose."""
    names = tracked_names(settings)
    report = {'mode': settings.box_tracking, 'tracked': names, 'boxes': {}}
    if not names:
        return settings, report
    measurements = measure_frames(frames, observations, settings)
    boxes = dict(settings.boxes)
    for name in names:
        moved, reason = relocate(boxes[name], measurements)
        if moved is None:
            report['boxes'][name] = {'updated': False, 'reason': reason}
            continue
        shift = float(np.linalg.norm(np.subtract(moved['floor_center_world_mm'][:2], boxes[name]['floor_center_world_mm'][:2])))
        report['boxes'][name] = {'updated': True, 'shift_mm': round(shift, 1),
                                 'rotation_deg': round(moved['world_yaw_deg']-boxes[name]['world_yaw_deg'], 2),
                                 'floor_center_world_mm': moved['floor_center_world_mm']}
        boxes[name] = moved
    import dataclasses
    return dataclasses.replace(settings, boxes=boxes), report
