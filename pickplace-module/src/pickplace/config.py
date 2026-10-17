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


def _obstacles(value):
    if value is None:
        return Settings.__dataclass_fields__['workcell_obstacles'].default_factory()
    if not isinstance(value, list) or not all(isinstance(o, dict) and isinstance(o.get('frame'), str) and isinstance(o.get('dims_mm'), list)
                                              and len(o['dims_mm']) == 3 and all(isinstance(v, (int, float)) and v > 0 for v in o['dims_mm']) for o in value):
        raise ValueError('workcell_obstacles must be [{"frame": name, "dims_mm": [x, y, z]}, ...]')
    return [{'frame': o['frame'], 'dims_mm': [float(v) for v in o['dims_mm']]} for o in value]


def _camera_correction(value):
    if value is None:
        return None
    m = value.get('matrix') if isinstance(value, dict) else None
    if not (isinstance(m, list) and len(m) == 4 and all(isinstance(r, list) and len(r) == 4 for r in m)):
        raise ValueError('camera_correction needs {"depth_scale": k, "matrix": 4x4}')
    k = _number(value, 'depth_scale', 1.0, .8, 1.2)
    return {'depth_scale': k, 'matrix': [[float(v) for v in r] for r in m]}


def _joints(value, label):
    if not isinstance(value, list) or len(value) != 6 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ValueError(label+' needs six finite joint angles (degrees)')
    return [float(v) for v in value]


@dataclass
class Settings:
    calibration: dict
    overview_joints_deg: list
    boxes: dict  # name -> accepted box reference; every box is also a collision obstacle
    # Boxes entered from the side (open face toward the robot, e.g. a shelf on the wall): the cube
    # is turned level and slid in horizontally. name -> spec (see _side_boxes).
    side_boxes: dict = field(default_factory=dict)
    home: dict | None = None  # {"xy_mm": [x, y], "yaw_deg": a}: the cube's table spot, goal "home"
    ring_spot: dict | None = None  # {"xy_mm", "diameter_mm", "height_mm"}: fixed ring; else vision finds it
    default_goal: str = 'ring'
    cube_size_mm: float = 200.0
    table_z_mm: float = -23.0
    # Pickup: approach above the cube, descend to pre-contact, switch suction on,
    # then press press_depth_mm below the taught contact height. Suction is open loop.
    press_depth_mm: float = 10.0
    pre_contact_mm: float = 20.0
    suction_dwell_s: float = 1.0  # the longest wait for the holding sensor to report a seal
    release_dwell_s: float = 1.0
    pick_retries: int = 3  # re-picks when the holding sensor shows no seal or a drop at the lift
    seal_settle_s: float = 0.5  # extra time on the cube after the sensor first reports a seal
    clearance_mm: float = 100.0
    # Machine scene obstacles the carried cube must clear. Poses are read live from the frame
    # system every cycle (recalibrations apply at once); dims are the components' box sizes.
    # The builtin planner does not check a WorldState-attached cube against frame-system
    # geometry, so these are added to the plan request explicitly.
    workcell_obstacles: list = field(default_factory=lambda: [
        {'frame': 'wall-front', 'dims_mm': [100.0, 3000.0, 483.0]},
        {'frame': 'wall-front-2', 'dims_mm': [100.0, 3000.0, 1277.0]},
        {'frame': 'ceiling', 'dims_mm': [3000.0, 3000.0, 100.0]},
        {'frame': 'box-shell-floor', 'dims_mm': [349.0, 286.0, 10.0]}, {'frame': 'box-shell-top', 'dims_mm': [349.0, 286.0, 10.0]},
        {'frame': 'box-shell-back', 'dims_mm': [10.0, 286.0, 254.0]}, {'frame': 'box-shell-left', 'dims_mm': [349.0, 10.0, 254.0]},
        {'frame': 'box-shell-right', 'dims_mm': [349.0, 10.0, 254.0]}])
    carry_clearance_mm: float = 50.0  # planner keeps this from those obstacles; the sweep check half of it
    carry_upright_deg: float = 15.0  # carried joint moves keep the tool within this of vertical (no cube swing)
    box_entry_clearance_mm: float = 30.0
    # Camera sweep when the cube is not fully seen from the overview: sideways shifts (dx, dy mm)
    # of the overview camera at the same height and orientation, and the sweep speed.
    sweep_offsets_mm: list = field(default_factory=lambda: [[0.0, 120.0], [0.0, 240.0], [0.0, 330.0], [0.0, -120.0], [100.0, 0.0]])
    sweep_speed_deg_s: float = 15.0
    release_above_goal_mm: float = 0.0
    grasp_toward_image_top_mm: float = 0.0
    placement_correction_world_xy_mm: list = field(default_factory=lambda: [0.0, 0.0])
    placement_world_yaw_deg: float = 0.0
    travel_speed_deg_s: float = 10.0
    approach_speed_deg_s: float = 3.0
    press_speed_deg_s: float = 2.0
    acceleration_deg_s2: float = 6.0
    joint_limit_margin_deg: float = 5.0
    camera_mount_reference: dict | None = None
    # Replacement camera -> the calibrated one: its depth is scaled by depth_scale, then points in
    # its frame map into the calibrated camera's frame by `matrix` (4x4). The task calibration
    # (overview fit, box, ring) then stays valid. Fit from fixed table features; see DEPLOYMENT.md.
    camera_correction: dict | None = None
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
        home = _home(attrs.get('home'))
        ring_spot = _ring(attrs.get('ring'))
        if home is not None and 'home' in boxes:
            raise ValueError('"home" is the table spot; give the box another name')
        goal = attrs.get('default_goal', 'ring')
        if goal not in ['ring', *boxes, *(['home'] if home else [])]:
            raise ValueError('default_goal must be "ring", "home" or a configured box name: '+', '.join(boxes))
        correction = attrs.get('placement_correction_world_xy_mm', [0, 0])
        if not isinstance(correction, list) or len(correction) != 2 or math.hypot(*correction) > 25:
            raise ValueError('placement_correction_world_xy_mm needs two values with magnitude <= 25 mm')
        mount = attrs.get('camera_mount_reference')
        s = cls(
            calibration=cal, overview_joints_deg=overview, boxes=boxes, side_boxes=_side_boxes(attrs.get('side_boxes'), boxes), home=home, ring_spot=ring_spot, default_goal=goal,
            cube_size_mm=_number(attrs, 'cube_size_mm', 200, 20, 400),
            table_z_mm=_number(attrs, 'table_z_mm', -23, -500, 500),
            press_depth_mm=_number(attrs, 'press_depth_mm', 10, 0, 20),
            pre_contact_mm=_number(attrs, 'pre_contact_mm', 20, 5, 100),
            suction_dwell_s=_number(attrs, 'suction_dwell_s', 1.0, 0, 10),
            release_dwell_s=_number(attrs, 'release_dwell_s', 1.0, 0, 10),
            pick_retries=int(_number(attrs, 'pick_retries', 3, 0, 5)),
            seal_settle_s=_number(attrs, 'seal_settle_s', 0.5, 0, 5),
            clearance_mm=_number(attrs, 'clearance_mm', 100, 30, 300),
            workcell_obstacles=_obstacles(attrs.get('workcell_obstacles')),
            carry_clearance_mm=_number(attrs, 'carry_clearance_mm', 50, 0, 200),
            carry_upright_deg=_number(attrs, 'carry_upright_deg', 15, 2, 90),
            box_entry_clearance_mm=_number(attrs, 'box_entry_clearance_mm', 30, 10, 200),
            release_above_goal_mm=_number(attrs, 'release_above_goal_mm', 0, 0, 50),
            sweep_offsets_mm=_offsets(attrs.get('sweep_offsets_mm', [[0, 120], [0, 240], [0, 330], [0, -120], [100, 0]])),
            sweep_speed_deg_s=_number(attrs, 'sweep_speed_deg_s', 15, 1, 100),
            grasp_toward_image_top_mm=_number(attrs, 'grasp_toward_image_top_mm', 0, 0, 20),
            placement_correction_world_xy_mm=[float(v) for v in correction],
            placement_world_yaw_deg=_number(attrs, 'placement_world_yaw_deg', 0, -180, 180),
            travel_speed_deg_s=_number(attrs, 'travel_speed_deg_s', 10, 0.5, 100),
            approach_speed_deg_s=_number(attrs, 'approach_speed_deg_s', 3, 0.5, 50),
            press_speed_deg_s=_number(attrs, 'press_speed_deg_s', 2, 0.2, 5),
            acceleration_deg_s2=_number(attrs, 'acceleration_deg_s2', 6, 1, 200),
            joint_limit_margin_deg=_number(attrs, 'joint_limit_margin_deg', 5, 1, 30),
            camera_mount_reference=mount if isinstance(mount, dict) else None,
            camera_correction=_camera_correction(attrs.get('camera_correction')),
            arm=_name(attrs, 'arm', 'arm'), gripper=_name(attrs, 'gripper', 'Vgripper'),
            camera=_name(attrs, 'camera', 'cam'), motion=_name(attrs, 'motion', 'builtin'),
            lease_service=_name(attrs, 'lease_service', 'code-1'),
        )
        if s.press_depth_mm >= s.pre_contact_mm+s.cube_size_mm/2:
            raise ValueError('press_depth_mm is unreasonably deep for this cube')
        return s

    @property
    def goals(self):
        """Every place the cube can go: the home table spot (if configured), the ring, each box."""
        return (['home'] if self.home else [])+['ring']+list(self.boxes)+list(self.side_boxes)

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
        elif place.get('type') in ('home', 'table'):
            goal = 'home'
        else:
            colour = place.get('color')
            matches = [n for n, b in [*self.boxes.items(), *self.side_boxes.items()] if colour is None or b.get('color') == str(colour).lower()]
            if place.get('type') not in (None, 'box') or not matches:
                raise ValueError('No configured place target matches '+str(place))
            goal = matches[0]
        if goal not in self.goals:
            raise ValueError('place goal must be one of: '+', '.join(self.goals))
        return goal

    def dependencies(self):
        names = [self.arm, self.gripper, self.camera, self.motion]
        return names+([self.lease_service] if self.lease_service else [])


def _home(raw):
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise ValueError('home must be {"xy_mm": [x, y], "yaw_deg": a}')
    xy = raw.get('xy_mm')
    yaw = raw.get('yaw_deg', 0)
    if not isinstance(xy, list) or len(xy) != 2 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in xy):
        raise ValueError('home.xy_mm needs two finite numbers (tool XY of the cube centre)')
    if isinstance(yaw, bool) or not isinstance(yaw, (int, float)) or not math.isfinite(yaw):
        raise ValueError('home.yaw_deg must be a finite number')
    return {'xy_mm': [float(v) for v in xy], 'yaw_deg': float(yaw)}


def _offsets(raw):
    ok = isinstance(raw, list) and len(raw) <= 8 and all(
        isinstance(p, list) and len(p) == 2 and all(isinstance(v, (int, float)) and not isinstance(v, bool) and abs(v) <= 400 for v in p) for p in raw)
    if not ok:
        raise ValueError('sweep_offsets_mm: up to 8 [dx, dy] camera shifts within +/-400 mm')
    return [[float(v) for v in p] for p in raw]


def _ring(raw):
    if raw is None:
        return None
    home = _home({'xy_mm': raw.get('xy_mm') if isinstance(raw, dict) else None})
    out = {'xy_mm': home['xy_mm']}
    for key, default in (('diameter_mm', 230.0), ('height_mm', 28.0)):
        v = raw.get(key, default)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not 0 < v < 600:
            raise ValueError(f'ring.{key} must be a positive number of mm')
        out[key] = float(v)
    return out


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


def _side_boxes(value, boxes):
    """`side_boxes: [{name, color, label, back_x_mm, floor_z_mm, center_y_mm, opening_y_mm: [lo, hi],
    top_z_mm, front_x_mm, panels: [frame names], back_clearance_mm, release_above_floor_mm}]`, all in
    the arm's world frame. The open face is toward -x (the robot)."""
    result = {}
    for i, raw in enumerate(value or []):
        name = raw.get('name') if isinstance(raw, dict) else None
        if not isinstance(name, str) or not name or name in ('ring', 'home') or name in boxes or name in result:
            raise ValueError(f'side_boxes[{i}] needs a unique name')
        b = {'name': name, 'color': str(raw.get('color', '')).lower() or None, 'label': raw.get('label', name),
             'panels': [str(f) for f in raw.get('panels', [])]}
        for key, lo, hi in (('back_x_mm', 200, 1200), ('floor_z_mm', -100, 1300), ('center_y_mm', -800, 800), ('top_z_mm', -100, 1500),
                            ('front_x_mm', 100, 1200)):
            b[key] = _number(raw, key, None, lo, hi) if key in raw else None
            if b[key] is None:
                raise ValueError(f'side_boxes.{name}.{key} is required')
        y = raw.get('opening_y_mm')
        if not (isinstance(y, list) and len(y) == 2 and y[0] < y[1]):
            raise ValueError(f'side_boxes.{name}.opening_y_mm needs [low, high]')
        b['opening_y_mm'] = [float(y[0]), float(y[1])]
        b['back_clearance_mm'] = _number(raw, 'back_clearance_mm', 25, 0, 200)
        b['release_above_floor_mm'] = _number(raw, 'release_above_floor_mm', 6, 0, 30)
        b['entry_margin_mm'] = _number(raw, 'entry_margin_mm', 15, 0, 200)
        b['roll_deg'] = _number(raw, 'roll_deg', 0, -10, 10)
        result[name] = b
    return result


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
