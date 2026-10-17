"""Measure a new orange Post-it box from one stationary RGB-D view (no motion).

Ported unchanged from the survey that produced the accepted "box" reference
(measure_box_3d): Post-it notes identify the box, depth gives four (possibly leaning)
wall planes and the floor, and the opening is the cavity common over the full depth.
Output is a CANDIDATE config entry: check it with a tape measure before adding it to
`boxes`. The camera calibration is only validated at the overview pose.
"""
import math

import cv2
import numpy as np

from .calibration import calibrated_xy, check_view
from .vision_geometry import fit_horizontal_plane, intrinsics, native, world_cloud

WORK_WIDTH = 800        # frames are resized to this width so the thresholds below hold at any resolution

BOX_HUE = (11, 30)      # cardboard brown, in OpenCV hue (0-179); skin sits a little lower (~8-13)
BOX_MIN_SAT = 60        # Otsu picks the saturation cutoff, but never below this
BOX_MIN_VAL = 40        # very dark pixels are shadows or the black ring, not cardboard
BOX_MIN_AREA_FRAC = 0.01  # outlines smaller than this fraction of the image are ignored
BOX_MIN_RECT_FILL = 0.85  # a box's (convex) outline fills its rotated rectangle; an arm or hand doesn't
BOX_MIN_RECT_FILL_CUT = 0.5  # looser for a box cut off by the image edge, where only part of it shows


def find_boxes(bgr: np.ndarray):
    scale = WORK_WIDTH / bgr.shape[1]
    small = cv2.resize(bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(cv2.GaussianBlur(small, (7, 7), 0), cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    otsu, _ = cv2.threshold(s, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    mask = ((s >= max(otsu, BOX_MIN_SAT)) & (h >= BOX_HUE[0]) & (h <= BOX_HUE[1]) & (v >= BOX_MIN_VAL))
    mask = mask.astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))  # bridge labels and print

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = BOX_MIN_AREA_FRAC * small.shape[0] * small.shape[1]
    boxes = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        (cx, cy), (rw, rh), angle = cv2.minAreaRect(c)
        x, y, w, hh = cv2.boundingRect(c)
        cut_off = x <= 1 or y <= 1 or x + w >= small.shape[1] - 1 or y + hh >= small.shape[0] - 1
        # convex outline, so a label or print on the box doesn't count as a gap in it
        fill = cv2.contourArea(cv2.convexHull(c)) / max(1.0, rw * rh)
        if fill < (BOX_MIN_RECT_FILL_CUT if cut_off else BOX_MIN_RECT_FILL):
            continue
        boxes.append({
            "center": (int(cx / scale), int(cy / scale)),
            "box": (cv2.boxPoints(((cx, cy), (rw, rh), angle)) / scale).astype(np.int32),
            "size": (int(rw / scale), int(rh / scale)),
            "angle": (angle + 45) % 90 - 45,
            "area": area / scale ** 2,
            "cut_off": cut_off,  # partly outside the picture: its center is only the visible part's center
            "contour": (c / scale).astype(np.int32),
        })
    return sorted(boxes, key=lambda b: b["area"], reverse=True)


def orange_markers(bgr):
    h=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
    # Separate saturated paper from dark brown cardboard. The older broad mask
    # merged a Post-it with the wall behind it under the hackathon lighting.
    mask=((h[:,:,0]>=5)&(h[:,:,0]<=28)&(h[:,:,1]>=185)&(h[:,:,2]>=75)).astype(np.uint8)*255
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    result=[]
    for c in contours:
        area=cv2.contourArea(c)
        if area<800 or area>.02*bgr.shape[0]*bgr.shape[1]:continue
        hull=cv2.convexHull(c);poly=cv2.approxPolyDP(hull,.04*cv2.arcLength(hull,True),True)
        rect=cv2.minAreaRect(c);a,b=rect[1]
        # A square note on a near-vertical wall projects to a thin quadrilateral.
        # Identity still requires two separate notes and measured cavity walls.
        if len(poly)!=4 or min(a,b)<12 or max(a,b)/min(a,b)>8 or area/max(a*b,1)<.6:continue
        result.append({'center':list(rect[0]),'area':area})
    return result


def fit_wall_planes(points, floor_z, seed=13):
    points=np.asarray(points,dtype=float)
    points=points[np.all(np.isfinite(points),axis=1)]
    points=points[(points[:,2]>floor_z+15)&(points[:,2]<floor_z+350)]
    points=points[::max(1,len(points)//6500)]
    remaining=points.copy();planes=[];rng=np.random.default_rng(seed)
    for _ in range(10):
        if len(remaining)<80:break
        best=None
        for _ in range(700):
            sample=remaining[rng.choice(len(remaining),3,replace=False)]
            normal=np.cross(sample[1]-sample[0],sample[2]-sample[0])
            length=np.linalg.norm(normal)
            if length<100:continue
            normal/=length
            # Cardboard walls may lean. Horizontal floors/flaps are separate.
            if abs(normal[2])>math.sin(math.radians(30)):continue
            offset=float(normal@sample[0]);mask=np.abs(remaining@normal-offset)<2.5
            count=int(mask.sum())
            if best is None or count>best[0]:best=(count,normal,offset,mask)
        if best is None or best[0]<80:break
        _,normal,offset,mask=best;support=remaining[mask]
        center=np.mean(support,axis=0)
        _,_,V=np.linalg.svd(support-center,full_matrices=False);normal=V[-1]
        if normal@best[1]<0:normal=-normal
        offset=float(normal@center)
        mask=np.abs(remaining@normal-offset)<2.5;support=remaining[mask]
        remaining=remaining[~mask]
        if len(support)<80 or np.ptp(support[:,2])<30:continue
        if abs(normal[2])>math.sin(math.radians(30)):continue
        planes.append({'normal':normal,'offset':offset,'support':support,
                       'rms_mm':float(np.sqrt(np.mean((support@normal-offset)**2)))})
    return planes


def measure_box(bgr,depth,props,pose):
    cloud=world_cloud(depth,intrinsics(props,depth.shape),pose);markers=orange_markers(bgr)
    candidates=[b for b in find_boxes(bgr) if any(cv2.pointPolygonTest(b['contour'],tuple(m['center']),False)>=0 for m in markers)]
    if len(candidates)!=1:raise ValueError('Need one depth-supported orange-marked box')
    mask=np.zeros(depth.shape,np.uint8);cv2.fillConvexPoly(mask,cv2.convexHull(candidates[0]['contour']),1)
    valid=(depth>150)&(depth<1500)&(mask>0)
    points=cloud[valid];n,d,floor=fit_horizontal_plane(points,tolerance=3)
    floor_z=float(np.median(floor[:,2]));planes=fit_wall_planes(points,floor_z)
    # Paired normals need not be exactly parallel: flexible cardboard leans.
    strongest=sorted(planes,key=lambda p:len(p['support']),reverse=True)
    if len(strongest)<4:raise ValueError('Need four observed wall planes')
    a=(math.atan2(strongest[0]['normal'][1],strongest[0]['normal'][0])+math.pi/4)%(math.pi/2)-math.pi/4
    R=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    groups=[]
    for axis in range(2):
        eligible=[p for p in strongest if abs(p['normal'][:2]@R[:,axis])/np.linalg.norm(p['normal'][:2])>math.cos(math.radians(8))]
        pair=[]
        for p in eligible:
            v=float(np.median(p['support'][:,:2]@R[:,axis]))
            if not pair or all(abs(v-u)>100 for u,_ in pair):pair.append((v,p))
            if len(pair)==2:break
        if len(pair)!=2:raise ValueError('A wall pair is unobserved')
        groups.append(sorted(pair,key=lambda x:x[0]))
    selected=[p for pair in groups for _,p in pair]
    rims=[float(np.percentile(p['support'][:,2],99)) for p in selected]
    rim=max(rims)
    # Find the largest axis-aligned cavity common to the wall planes over the
    # entire insertion depth. Leaning walls are evaluated at both heights.
    center=np.array([np.mean([v for v,p in pair]) for pair in groups])
    bounds=[]
    for axis,pair in enumerate(groups):
        values=[]
        for side,(_,p) in enumerate(pair):
            normal=p['normal'];local=normal[:2]@R;other=1-axis
            endpoints=[(p['offset']-normal[2]*z-local[other]*center[other])/local[axis] for z in [floor_z,rim]]
            values.append(max(endpoints) if side==0 else min(endpoints))
        bounds.append(values)
    low=np.array([v[0] for v in bounds]);high=np.array([v[1] for v in bounds]);center=(low+high)/2
    # Include each wall's in-plane skew over the candidate rectangle too.
    for _ in range(2):
        for axis,pair in enumerate(groups):
            for side,(_,p) in enumerate(pair):
                normal=p['normal'];local=normal[:2]@R;other=1-axis
                vals=[(p['offset']-normal[2]*z-local[other]*w)/local[axis] for z in [floor_z,rim] for w in [low[other],high[other]]]
                if side==0:low[axis]=max(vals)
                else:high[axis]=min(vals)
        center=(low+high)/2
    dims=high-low
    if min(dims)<100 or max(dims)>700:raise ValueError('Invalid observed cavity')
    xy=R@center;fz=float((d-n[:2]@xy)/n[2])
    floor_inside=floor[((floor[:,:2]@R>low+10)&(floor[:,:2]@R<high-10)).all(axis=1)]
    if len(floor_inside)<200:raise ValueError('Inside floor not observed')
    return native({'identity':'orange_postit_box','floor_center_world_mm':[*xy,fz],'inner_dimensions_mm':dims,'world_yaw_deg':math.degrees(a),'rim_z_mm':rim,
        'cavity_depth_mm':rim-fz,'floor_normal_world':n,'floor_samples':len(floor_inside),'marker_count':len(markers),'measurement_margin_mm':0,
        'wall_support':[len(p['support']) for p in selected],'wall_rms_mm':[p['rms_mm'] for p in selected],
        'wall_planes':[{'normal':p['normal'],'offset':p['offset']} for p in selected],
        'opening_corners_world_mm':[[*(R@np.array([x,y])),rim] for x,y in [(low[0],low[1]),(high[0],low[1]),(high[0],high[1]),(low[0],high[1])]],
        'scope':'Common opening over measured wall height; no dimensions substituted'})


def candidate_entry(name, measured, calibration, observed_table_z, cube_size):
    """Camera-world measurement -> config entry in tool coordinates.

    XY uses the overview calibration. Heights are shifted so the observed tabletop matches
    the taught one (taught pickup contact - cube size), the same vertical reference the
    cube uses.
    """
    dz = calibration['pickup_tcp_z_mm']-cube_size-observed_table_z
    floor = measured['floor_center_world_mm']
    xy = calibrated_xy(calibration, floor)
    return native({'name': name, 'floor_center_world_mm': [*xy, floor[2]+dz], 'placement_center_xy_mm': list(xy),
                   'inner_dimensions_mm': measured['inner_dimensions_mm'],
                   'world_yaw_deg': measured['world_yaw_deg']+calibration['yaw_correction_deg'],
                   'rim_z_mm': measured['rim_z_mm']+dz})


def survey(bgr, depth, props, camera_pose, observation, calibration, name, cube_size):
    """One frame -> {'candidate': entry} or {'issues': [...]}. observation is the cube/ring analysis."""
    check_view(observation, calibration)
    try:
        measured = measure_box(bgr, depth, props, camera_pose)
    except ValueError as e:
        return {'issues': [str(e)]}
    table_z = observation.get('observed_table_z_mm')
    if table_z is None:
        return {'issues': ['Tabletop not measured in this view']}
    return {'candidate': candidate_entry(name, measured, calibration, table_z, cube_size), 'raw': measured}
