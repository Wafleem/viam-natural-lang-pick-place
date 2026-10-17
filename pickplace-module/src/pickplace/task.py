"""Pick and place target geometry for one cycle (pure math, no robot calls).

Vision gives the cube pose; the overview calibration maps it to tool coordinates.
The cube is square, so pickup and placement yaw are free in 90 deg steps; the step
chosen is the one with the least joint travel and full joint-limit margin (IK).
"""
import copy
import math

import numpy as np

from .box import check_cube_fits, placement_xy
from .calibration import cube_target, ring_target
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


def _travel(a, b):
    return sum(abs(x-y) for x, y in zip(a, b))


def compute_targets(settings, observation, goal, kinematics, start_joints_deg):
    """All tool poses for one cycle plus IK joint goals for the free-space moves."""
    if goal not in settings.goals:
        raise ValueError('goal must be one of: '+', '.join(settings.goals))
    s, cal, margin = settings, settings.calibration, settings.joint_limit_margin_deg
    cube_top, detected_yaw = cube_target(observation, cal)
    size = s.cube_size_mm
    box = s.boxes.get(goal)

    if box is not None:
        goal_top = np.array([*placement_xy(box), box['floor_center_world_mm'][2]+size])
        place_base_yaw = box['world_yaw_deg']
    else:
        goal_top = ring_target(observation, cal)+np.array([*s.placement_correction_world_xy_mm, 0.])
        place_base_yaw = s.placement_world_yaw_deg

    pick_z, place_z = cube_top[2], goal_top[2]
    # Approach heights, each side on its own. A shared, high carry height (rim + cube + 100 mm)
    # pushed the wrist onto the J4 ~ 0/360 branch; the taught J4 ~ 180 branch reaches about
    # 360 mm above the cube. The planner routes the transfer around boxes with the carried cube.
    above_pick_z = pick_z+s.clearance_mm
    above_place_z = place_z+s.clearance_mm
    if box is not None:
        # Carried cube bottom (TCP - cube size) clears this box's rim before the vertical entry.
        above_place_z = max(above_place_z, box['rim_z_mm']+size+s.box_entry_clearance_mm)

    # Pickup yaw: four equivalent square orientations; keep the one nearest the start.
    best = None
    for k in range(4):
        yaw = detected_yaw+90*k
        offset = grasp_offset(observation, cal, yaw, s.grasp_toward_image_top_mm)
        contact = downward_pose(cube_top+offset, yaw)
        above = {**contact, 'z': above_pick_z}
        try:
            q = kinematics.ik(above, start_joints_deg, margin)
            kinematics.ik({**contact, 'z': pick_z-s.press_depth_mm}, q, margin)
        except ValueError:
            continue
        if best is None or _travel(q, start_joints_deg) < best[0]:
            best = (_travel(q, start_joints_deg), yaw, contact, above, q)
    if best is None:
        raise ValueError('No pickup orientation is reachable with the joint-limit margin')
    _, pick_yaw, contact, above_pick, q_above_pick = best
    grasp = describe_grasp(cube_top, contact, pick_yaw, observation['cube']['top_dimensions_mm'])

    best = None
    for k in range(4):
        place_yaw = place_base_yaw+90*k
        if box is not None:
            check_cube_fits(box, size, goal_top, place_yaw)
        release = place_tcp(goal_top, place_yaw, grasp)
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
        'goal': goal,
        'cube_top_world_mm': cube_top.tolist(), 'cube_yaw_deg': pick_yaw,
        'goal_top_world_mm': goal_top.tolist(), 'place_yaw_deg': place_yaw,
        'above_pick': above_pick, 'pre_contact': {**contact, 'z': pick_z+s.pre_contact_mm},
        'contact': contact, 'press': {**contact, 'z': pick_z-s.press_depth_mm},
        'release': release, 'above_place': above_place,
        'q_above_pick': q_above_pick, 'q_above_place': q_above_place,
        'grasp': copy.deepcopy(grasp),
    }
