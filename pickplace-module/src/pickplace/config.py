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
    boxes: dict  # name -> accepted box reference; every box is also a collision obstacle
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
        boxes = _boxes(attrs)
        goal = attrs.get('default_goal', 'ring')
        if goal != 'ring' and goal not in boxes:
            raise ValueError('default_goal must be "ring" or a configured box name: '+', '.join(boxes))
        correction = attrs.get('placement_correction_world_xy_mm', [0, 0])
        if not isinstance(correction, list) or len(correction) != 2 or math.hypot(*correction) > 25:
            raise ValueError('placement_correction_world_xy_mm needs two values with magnitude <= 25 mm')
        mount = attrs.get('camera_mount_reference')
        s = cls(
            calibration=cal, overview_joints_deg=overview, boxes=boxes, default_goal=goal,
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

    @property
    def goals(self):
        return ['ring']+list(self.boxes)

    def resolve_place(self, place):
        """Web/NL place target -> goal name. Accepts "ring", a box name, or
        {"goal": name} / {"type": "ring"|"box"} / {"color": "orange"} (first match)."""
        if isinstance(place, str):
            place = {'goal': place}
        if not isinstance(place, dict):
            raise ValueError('place must be a goal name or an object')
        if place.get('goal') is not None:
            goal = place['goal']
        elif place.get('type') == 'ring':
            goal = 'ring'
        else:
            colour = place.get('color')
            matches = [n for n, b in self.boxes.items() if colour is None or b.get('color') == str(colour).lower()]
            if place.get('type') not in (None, 'box') or not matches:
                raise ValueError('No configured place target matches '+str(place))
            goal = matches[0]
        if goal not in self.goals:
            raise ValueError('place goal must be one of: '+', '.join(self.goals))
        return goal

    def dependencies(self):
        names = [self.arm, self.gripper, self.camera, self.motion]
        return names+([self.lease_service] if self.lease_service else [])


BOX_KEYS = ('floor_center_world_mm', 'placement_center_xy_mm', 'inner_dimensions_mm', 'world_yaw_deg', 'rim_z_mm')


def _box(raw, label):
    """One accepted box: floor centre (x,y,z), opening (2), yaw, rim z; optional placement XY."""
    if not isinstance(raw, dict):
        raise ValueError(label+' must be an object')
    def vector(key, n):
        v = raw.get(key)
        if not isinstance(v, list) or len(v) != n or any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in v):
            raise ValueError(f'{label}.{key} needs {n} finite numbers')
        return [float(x) for x in v]
    box = {'floor_center_world_mm': vector('floor_center_world_mm', 3), 'inner_dimensions_mm': vector('inner_dimensions_mm', 2)}
    if 'placement_center_xy_mm' in raw:
        box['placement_center_xy_mm'] = vector('placement_center_xy_mm', 2)
    for key in ('world_yaw_deg', 'rim_z_mm'):
        v = raw.get(key)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ValueError(f'{label}.{key} must be a finite number')
        box[key] = float(v)
    for key in ('color', 'label'):
        if key in raw:
            if not isinstance(raw[key], str) or not raw[key]:
                raise ValueError(f'{label}.{key} must be a non-empty string')
            box[key] = raw[key].lower() if key == 'color' else raw[key]
    box.setdefault('color', 'orange')
    if min(box['inner_dimensions_mm']) <= 0 or box['rim_z_mm'] <= box['floor_center_world_mm'][2]:
        raise ValueError(label+' needs a positive opening and a rim above its floor')
    return box


def _boxes(attrs):
    """`boxes: [{name, ...}]`, or the single legacy `box` / `surveyed_box.box` (named "box")."""
    result = {}
    if attrs.get('boxes') is not None:
        if not isinstance(attrs['boxes'], list):
            raise ValueError('boxes must be a list of {name, floor_center_world_mm, ...}')
        for i, raw in enumerate(attrs['boxes']):
            name = raw.get('name') if isinstance(raw, dict) else None
            if not isinstance(name, str) or not name or name == 'ring' or name in result:
                raise ValueError(f'boxes[{i}] needs a unique name other than "ring"')
            result[name] = _box(raw, 'boxes.'+name)
        return result
    legacy = attrs.get('box')
    if legacy is None and isinstance(attrs.get('surveyed_box'), dict):
        legacy = attrs['surveyed_box'].get('box')
    if legacy is not None:
        result['box'] = _box({k: legacy[k] for k in BOX_KEYS if k in legacy}, 'box')
    return result
