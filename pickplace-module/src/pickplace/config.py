"""Service attributes -> validated Settings.

Accepts the existing armfarm19 code-2 attributes unchanged (task_calibration,
surveyed_box.box, grasp_toward_image_top_mm, ...) so deploying this module does not
require rewriting the machine config. New, optional keys are documented in DEPLOYMENT.md.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


def _number(attrs, key, default, low, high):
    value = attrs.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{key} must be a number in [{low}, {high}]')
    return float(value)


def _name(attrs, key, default):
    value = attrs.get(key, default)
    if not isinstance(value, str):
        raise ValueError(key+' must be a string')
    return value


def _joints(value, label):
    if not isinstance(value, list) or len(value) != 6 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ValueError(label+' needs six finite joint angles (degrees)')
    return [float(v) for v in value]


@dataclass
class Settings:
    calibration: dict
    overview_joints_deg: list
    box: dict | None
    default_goal: str = 'ring'
    cube_size_mm: float = 200.0
    table_z_mm: float = -23.0
    # Pickup: approach above the cube, descend to pre-contact, switch suction on,
    # then press press_depth_mm below the taught contact height. Suction is open loop.
    press_depth_mm: float = 10.0
    pre_contact_mm: float = 20.0
    suction_dwell_s: float = 1.0
    release_dwell_s: float = 1.0
    clearance_mm: float = 100.0
    grasp_toward_image_top_mm: float = 0.0
    placement_correction_world_xy_mm: list = field(default_factory=lambda: [0.0, 0.0])
    placement_world_yaw_deg: float = 0.0
    travel_speed_deg_s: float = 10.0
    approach_speed_deg_s: float = 3.0
    press_speed_deg_s: float = 2.0
    acceleration_deg_s2: float = 6.0
    joint_limit_margin_deg: float = 5.0
    camera_mount_reference: dict | None = None
    arm: str = 'arm'
    gripper: str = 'Vgripper'
    camera: str = 'cam'
    motion: str = 'builtin'
    lease_service: str = 'code-1'

    @classmethod
    def from_attributes(cls, attrs):
        cal = attrs.get('task_calibration')
        keys = ('observation_joints_deg', 'yaw_correction_deg', 'xy_offset_mm', 'pickup_tcp_z_mm', 'place_tcp_z_mm', 'observed_cube_z_mm', 'observed_ring_z_mm')
        if not isinstance(cal, dict) or any(k not in cal for k in keys):
            raise ValueError('task_calibration with '+', '.join(keys)+' is required')
        overview = _joints(cal['observation_joints_deg'], 'task_calibration.observation_joints_deg')
        if attrs.get('aligned_depth_enabled') is not True:
            raise ValueError('Enable cam align_color_depth and set aligned_depth_enabled=true')
        box = attrs.get('box')
        if box is None and isinstance(attrs.get('surveyed_box'), dict):
            box = attrs['surveyed_box'].get('box')
        if box is not None:
            for key in ('floor_center_world_mm', 'inner_dimensions_mm', 'world_yaw_deg', 'rim_z_mm'):
                if key not in box:
                    raise ValueError('box.'+key+' is required')
            box = {k: box[k] for k in ('floor_center_world_mm', 'placement_center_xy_mm', 'inner_dimensions_mm', 'world_yaw_deg', 'rim_z_mm') if k in box}
        goal = attrs.get('default_goal', 'ring')
        if goal not in ('ring', 'box'):
            raise ValueError('default_goal must be ring or box')
        correction = attrs.get('placement_correction_world_xy_mm', [0, 0])
        if not isinstance(correction, list) or len(correction) != 2 or math.hypot(*correction) > 25:
            raise ValueError('placement_correction_world_xy_mm needs two values with magnitude <= 25 mm')
        mount = attrs.get('camera_mount_reference')
        s = cls(
            calibration=cal, overview_joints_deg=overview, box=box, default_goal=goal,
            cube_size_mm=_number(attrs, 'cube_size_mm', 200, 20, 400),
            table_z_mm=_number(attrs, 'table_z_mm', -23, -500, 500),
            press_depth_mm=_number(attrs, 'press_depth_mm', 10, 0, 20),
            pre_contact_mm=_number(attrs, 'pre_contact_mm', 20, 5, 100),
            suction_dwell_s=_number(attrs, 'suction_dwell_s', 1.0, 0, 10),
            release_dwell_s=_number(attrs, 'release_dwell_s', 1.0, 0, 10),
            clearance_mm=_number(attrs, 'clearance_mm', 100, 30, 300),
            grasp_toward_image_top_mm=_number(attrs, 'grasp_toward_image_top_mm', 0, 0, 20),
            placement_correction_world_xy_mm=[float(v) for v in correction],
            placement_world_yaw_deg=_number(attrs, 'placement_world_yaw_deg', 0, -180, 180),
            travel_speed_deg_s=_number(attrs, 'travel_speed_deg_s', 10, 0.5, 20),
            approach_speed_deg_s=_number(attrs, 'approach_speed_deg_s', 3, 0.5, 10),
            press_speed_deg_s=_number(attrs, 'press_speed_deg_s', 2, 0.2, 5),
            acceleration_deg_s2=_number(attrs, 'acceleration_deg_s2', 6, 1, 50),
            joint_limit_margin_deg=_number(attrs, 'joint_limit_margin_deg', 5, 1, 30),
            camera_mount_reference=mount if isinstance(mount, dict) else None,
            arm=_name(attrs, 'arm', 'arm'), gripper=_name(attrs, 'gripper', 'Vgripper'),
            camera=_name(attrs, 'camera', 'cam'), motion=_name(attrs, 'motion', 'builtin'),
            lease_service=_name(attrs, 'lease_service', 'code-1'),
        )
        if s.press_depth_mm >= s.pre_contact_mm+s.cube_size_mm/2:
            raise ValueError('press_depth_mm is unreasonably deep for this cube')
        return s

    def dependencies(self):
        names = [self.arm, self.gripper, self.camera, self.motion]
        return names+([self.lease_service] if self.lease_service else [])
