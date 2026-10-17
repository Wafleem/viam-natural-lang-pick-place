"""Fixed-overview task calibration: camera world XY -> taught tool XY, plus taught heights.

The calibration (fit on the armfarm19 workcell) holds for ONE wrist pose: the overview
joints it records. Objects must rest on the same tabletop. Heights come from taught
contacts (pickup_tcp_z_mm, place_tcp_z_mm), not from camera depth.
"""
import math

import numpy as np

from .transforms import pose_matrix


def _rotation(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def check_view(observation, calibration):
    """Camera pose and intrinsics must match the calibration capture."""
    joints = observation.get('observation_joints_deg', [])
    if len(joints) != 6 or max(abs(a-b) for a, b in zip(joints, calibration['observation_joints_deg'])) > .25:
        raise ValueError('Return to the calibrated overview joints before detecting targets')
    if calibration.get('camera_pose_world'):
        a = np.array(pose_matrix(calibration['camera_pose_world']))
        b = np.array(pose_matrix(observation['camera_pose_world']))
        angle = math.degrees(math.acos(np.clip((np.trace(a[:3, :3].T @ b[:3, :3])-1)/2, -1, 1)))
        if np.linalg.norm(a[:3, 3]-b[:3, 3]) > 3 or angle > .5:
            raise ValueError('Camera pose/configuration changed since calibration')
    if calibration.get('camera_intrinsics') and calibration['camera_intrinsics'] != observation.get('camera_intrinsics'):
        raise ValueError('Camera intrinsics changed since calibration')


def calibrated_xy(calibration, xy):
    return _rotation(calibration['yaw_correction_deg']) @ np.asarray(xy[:2], float)+np.asarray(calibration['xy_offset_mm'], float)


def cube_target(observation, calibration, yaw_step=90.0):
    """(object top center at taught contact height, world yaw in [-step/2, step/2)).

    yaw_step is the object's symmetry: 90 for the square cube, 180 for a phone (long axis)."""
    check_view(observation, calibration)
    cube = observation['cube']
    if abs(cube['top_center_world_mm'][2]-calibration['observed_cube_z_mm']) > 12:
        raise ValueError('Cube height changed; it must rest on the calibrated tabletop')
    xy = calibrated_xy(calibration, cube['top_center_world_mm'])
    yaw = (cube['world_yaw_deg']+calibration['yaw_correction_deg']+yaw_step/2) % yaw_step-yaw_step/2
    return np.array([*xy, calibration['pickup_tcp_z_mm']]), float(yaw)


def ring_target(observation, calibration):
    """Cube top center when seated on the ring (taught place height)."""
    check_view(observation, calibration)
    ring = observation.get('ring')
    if not ring:
        raise ValueError('Ring goal requires one full ring in view')
    if abs(ring['top_center_world_mm'][2]-calibration['observed_ring_z_mm']) > 12:
        raise ValueError('Ring height changed; it must rest on the calibrated tabletop')
    xy = calibrated_xy(calibration, ring['top_center_world_mm'])
    return np.array([*xy, calibration['place_tcp_z_mm']])


def calibrated_direction(calibration, xy):
    """A camera-world direction (unit xy) in tool coordinates: rotation only, no offset."""
    return _rotation(calibration['yaw_correction_deg']) @ np.asarray(xy[:2], float)
