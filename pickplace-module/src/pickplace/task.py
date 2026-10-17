"""Pick and place target geometry for one cycle (pure math, no robot calls).

Vision gives the cube pose; the overview calibration maps it to tool coordinates.
The cube is square, so pickup and placement yaw are free in 90 deg steps; the step
chosen is the one with the least joint travel and full joint-limit margin (IK).
"""
import copy
import math

import numpy as np

from .box import check_cube_fits, placement_xy
from .calibration import calibrated_direction, cube_target, ring_target
from .grasp_geometry import describe_grasp, place_tcp
from .transforms import pose_matrix
from .vision_geometry import downward_pose


def grasp_offset(observation, calibration, yaw, amount):
    """Shift the suction point `amount` mm toward the cube edge facing image-top.

    Kept from the validated ring cycles: the cup seals better away from the printed
    centre logo. Same result for every 90 deg yaw candidate.
    """
    if not amount:
        return np.zeros(3)
    up = -np.array(pose_matrix(observation['camera_pose_world']))[:2, 1]
    c = math.radians(calibration['yaw_correction_deg'])
    up = np.array([[math.cos(c), -math.sin(c)], [math.sin(c), math.cos(c)]]) @ up
    if np.linalg.norm(up) < .5:
        raise ValueError('Cannot determine upper face edge from this camera view')
    a = math.radians(yaw)
    u, v = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
    edge = max([u, -u, v, -v], key=lambda axis: float(axis @ up))
    return np.array([*(edge*amount), 0.])


def tool_yaw_aligning(axis_tool_deg, world_deg):
    """Tool-down yaw (downward_pose theta) that points the tool-frame direction axis_tool_deg
    along world_deg. Solved from two evaluations; the map is affine with slope +/-1."""
    def world_angle(t):
        R = np.array(pose_matrix(downward_pose(np.zeros(3), t)))[:3, :3]
        a = math.radians(axis_tool_deg)
        v = R @ np.array([math.cos(a), math.sin(a), 0.])
        return math.degrees(math.atan2(v[1], v[0]))
    a0 = world_angle(0.0)
    slope = 1.0 if abs(((world_angle(90.0)-a0)-90+180) % 360-180) < 1 else -1.0
    t = (world_deg-a0)/slope
    return (t+180) % 360-180


def phone_candidates(settings, observation, top, yaw):
    """(object yaw, contact TCP, grasp) for both 180 deg turns of a phone: cups along its length,
    the cup centre shifted grip_away_from_bump_mm away from the camera bump (if seen)."""
    s, phone = settings, observation['cube']
    offset = np.zeros(3)
    if phone.get('bump_world_xy') and s.grip_away_from_bump_mm:
        offset[:2] = -calibrated_direction(s.calibration, phone['bump_world_xy'])*s.grip_away_from_bump_mm
    result = []
    for k in range(2):
        obj_yaw = yaw+180*k
        contact = downward_pose(top+offset, tool_yaw_aligning(s.cup_axis_tool_deg, obj_yaw))
        result.append((obj_yaw, contact, describe_grasp(top, contact, obj_yaw, phone['top_dimensions_mm'])))
    return result


def _travel(a, b):
    return sum(abs(x-y) for x, y in zip(a, b))


def goal_surface(settings, observation, goal):
    """(cube top centre when seated at the goal, base yaw, box or None) for any place."""
    s, size = settings, settings.cube_size_mm
    box = s.boxes.get(goal)
    if box is not None:
        return np.array([*placement_xy(box), box['floor_center_world_mm'][2]+size]), box['world_yaw_deg'], box
    if goal == 'home':
        # A table spot: the cube rests on the calibrated tabletop (taught contact height).
        return np.array([*s.home['xy_mm'], s.calibration['pickup_tcp_z_mm']]), s.home['yaw_deg'], None
    if s.ring_spot:  # fixed ring: the cube centre goes exactly on the configured spot
        return np.array([*s.ring_spot['xy_mm'], s.calibration['place_tcp_z_mm']]), s.placement_world_yaw_deg, None
    return ring_target(observation, s.calibration)+np.array([*s.placement_correction_world_xy_mm, 0.]), s.placement_world_yaw_deg, None


def compute_targets(settings, observation, goal, kinematics, start_joints_deg, source=None):
    """All tool poses for one cycle plus IK joint goals for the free-space moves.

    source=None picks the cube vision found on the table. Otherwise source is the remembered
    cube location {place, top_world_mm, yaw_deg, tcp_in_cube} from an earlier placement: the cup
    returns to the same spot on the cube it held before (no vision needed inside a box).
    """
    if goal not in settings.goals:
        raise ValueError('goal must be one of: '+', '.join(settings.goals))
    s, cal, margin = settings, settings.calibration, settings.joint_limit_margin_deg
    size = s.cube_size_mm
    if source is None:
        cube_top, detected_yaw = cube_target(observation, cal, s.yaw_step_deg)
        source_place = 'table'
        candidates = phone_candidates(s, observation, cube_top, detected_yaw) if s.object_kind == 'phone' else []
        for k in range(0 if s.object_kind == 'phone' else 4):
            yaw = detected_yaw+90*k
            offset = grasp_offset(observation, cal, yaw, s.grasp_toward_image_top_mm)
            contact = downward_pose(cube_top+offset, yaw)
            candidates.append((yaw, contact, describe_grasp(cube_top, contact, yaw, observation['cube']['top_dimensions_mm'])))
    else:
        cube_top, source_place = np.asarray(source['top_world_mm'], float), source['place']
        G = {'tcp_in_cube': source['tcp_in_cube']}
        # Re-grip at any symmetric turn of the same cup spot: four for the cube, two for a phone.
        step = s.yaw_step_deg
        candidates = [(source['yaw_deg']+step*k, place_tcp(cube_top, source['yaw_deg']+step*k, G),
                       {**copy.deepcopy(source.get('grasp', {})), 'tcp_in_cube': source['tcp_in_cube']}) for k in range(int(360/step))]
    source_box = s.boxes.get(source_place)
    goal_top, place_base_yaw, box = goal_surface(s, observation, goal)

    pick_z, place_z = cube_top[2], goal_top[2]
    # Approach heights, each side on its own. A shared, high carry height (rim + cube + 100 mm)
    # pushed the wrist onto the J4 ~ 0/360 branch; the taught J4 ~ 180 branch reaches about
    # 360 mm above the cube. The planner routes the transfer around boxes with the carried cube.
    above_pick_z = pick_z+s.clearance_mm
    above_place_z = place_z+s.clearance_mm
    if source_box is not None:
        above_pick_z = max(above_pick_z, source_box['rim_z_mm']+size+s.box_entry_clearance_mm)
    if box is not None:
        # Carried cube bottom (TCP - cube size) clears this box's rim before the vertical entry.
        above_place_z = max(above_place_z, box['rim_z_mm']+size+s.box_entry_clearance_mm)

    # Pickup yaw: four equivalent square orientations; keep the one nearest the start.
    best = None
    for yaw, contact, grasp in candidates:
        above = {**contact, 'z': above_pick_z}
        try:
            q = kinematics.ik(above, start_joints_deg, margin)
            kinematics.ik({**contact, 'z': pick_z-s.press_depth_mm}, q, margin)
        except ValueError:
            continue
        # From memory, prefer the exact remembered grip (first candidate): same cup spot every time.
        cost = _travel(q, start_joints_deg)+(0 if source is None or not candidates.index((yaw, contact, grasp)) else 1e6)
        if best is None or cost < best[0]:
            best = (cost, yaw, contact, above, q, grasp)
    if best is None:
        raise ValueError('No pickup orientation is reachable with the joint-limit margin')
    _, pick_yaw, contact, above_pick, q_above_pick, grasp = best

    best = None
    for k in range(4):
        place_yaw = place_base_yaw+90*k
        if box is not None:
            check_cube_fits(box, s.object_footprint_mm, goal_top, place_yaw)
        release = place_tcp(goal_top, place_yaw, grasp)
        release['z'] += s.release_above_goal_mm  # optional drop height (default 0)
        above = {**release, 'z': above_place_z}
        try:
            q = kinematics.ik(above, q_above_pick, margin)
            kinematics.ik(release, q, margin)
        except ValueError:
            continue
        if best is None or _travel(q, q_above_pick) < best[0]:
            best = (_travel(q, q_above_pick), place_yaw, release, above, q)
    if best is None:
        raise ValueError('No placement orientation is reachable with the joint-limit margin')
    _, place_yaw, release, above_place, q_above_place = best

    return {
        'goal': goal, 'source_place': source_place,
        'cube_top_world_mm': cube_top.tolist(), 'cube_yaw_deg': pick_yaw,
        'goal_top_world_mm': goal_top.tolist(), 'place_yaw_deg': place_yaw,
        'above_pick': above_pick, 'pre_contact': {**contact, 'z': pick_z+s.pre_contact_mm},
        'contact': contact, 'press': {**contact, 'z': pick_z-s.press_depth_mm},
        'release': release, 'above_place': above_place,
        'q_above_pick': q_above_pick, 'q_above_place': q_above_place,
        'grasp': copy.deepcopy(grasp),
    }
