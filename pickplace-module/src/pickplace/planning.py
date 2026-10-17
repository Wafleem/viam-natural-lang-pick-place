"""Collision-checked planning through the Viam builtin motion service (plan only).

Every stage is planned before anything moves. The motion service's "plan" DoCommand
never moves the arm; execution happens separately, stage by stage, in cycle.py.
Free-space moves use IK joint goals chosen in task.py; vertical moves use straight-line
pose goals. Every returned joint node must keep joint_limit_margin_deg from its limit.
"""
import asyncio
import json
import math

import numpy as np
from google.protobuf.json_format import MessageToDict
from viam.proto.common import Geometry, GeometriesInFrame, Pose, PoseInFrame, RectangularPrism, Transform, Vector3, WorldState
from viam.proto.service.motion import CollisionSpecification, Constraints, LinearConstraint, OrientationConstraint

from .box import cavity_geometry, floor_label
from .transforms import pose_matrix

CUBE, RING, TABLE = 'vision-cube', 'vision-ring', 'vision-table'
# Getting back to the overview may start anywhere; it only has to stay off the hard bound.
RESET_MARGIN_DEG = 1.0

# name, motion kind, speed key, cube state, allowed contacts
STAGES = [
    ('approach', 'joint', 'travel', 'at_pick', 'pickup'),
    ('descend', 'straight', 'approach', 'at_pick', 'pickup'),
    ('press', 'straight', 'press', 'at_pick', 'pickup'),
    ('lift', 'straight', 'approach', 'carried', 'pickup'),
    ('transfer', 'joint', 'travel', 'carried', 'none'),
    ('place', 'straight', 'approach', 'carried', 'place'),
    ('retract', 'straight', 'approach', 'placed', 'place'),
    ('return', 'joint', 'travel', 'placed', 'place'),
]


def prism(label, xyz, dims, yaw=0.0):
    return Geometry(label=label, center=Pose(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]), o_z=1, theta=float(yaw)),
                    box=RectangularPrism(dims_mm=Vector3(x=float(dims[0]), y=float(dims[1]), z=float(dims[2]))))


def workcell_prisms(workcell, inflate=0.0, exact=()):
    """[(frame, center_xyz, dims, yaw_deg)] -> obstacle geometries (labels scene-<frame>), grown by
    `inflate` mm on every side so the planner itself keeps that clearance. Frames in `exact` (the
    panels of a side box the cube is entering) keep their true size."""
    return [prism('scene-'+name, xyz, [d+(0 if name in exact else 2*inflate) for d in dims], yaw) for name, xyz, dims, yaw in workcell]


def carried_cube_violations(kin, path, tcp_in_cube, size, workcell, clearance, steps=12):
    """Sweep the carried cube along a joint path (linear joint interpolation, as executed) and
    return [(obstacle, min signed gap mm)] for every workcell box it comes within `clearance` of."""
    cube_from_tool = np.linalg.inv(np.asarray(tcp_in_cube, float))
    h = size/2
    pts = np.array([[x, y, z, 1.0] for x in (-h, 0, h) for y in (-h, 0, h) for z in (0, -size/2, -size)])
    worst = {}
    for a, b in zip(path[:-1], path[1:]):
        for t in np.linspace(0, 1, steps):
            q = [x+(y-x)*t for x, y in zip(a, b)]
            world = (kin.fk(q) @ cube_from_tool @ pts.T).T[:, :3]
            for name, xyz, dims, yaw in workcell:
                c, sn = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
                d = world-np.asarray(xyz, float)
                local = np.c_[d[:, 0]*c+d[:, 1]*sn, -d[:, 0]*sn+d[:, 1]*c, d[:, 2]]
                gap = float(np.min(np.max(np.abs(local)-np.asarray(dims)/2, axis=1)))  # <0: inside
                if gap < clearance:
                    worst[name] = min(gap, worst.get(name, gap))
    return sorted(worst.items())


def max_tool_tilt_deg(kin, path, steps=12):
    """Largest angle between the tool axis and straight down along a joint path."""
    worst = 0.0
    for a, b in zip(path[:-1], path[1:]):
        for t in np.linspace(0, 1, steps):
            z = kin.fk([x+(y-x)*t for x, y in zip(a, b)])[:3, 2]
            worst = max(worst, math.degrees(math.acos(float(np.clip(-z[2], -1, 1)))))
    return worst


def scene_world(settings, observation, targets, cube_state, goal, workcell=()):
    """WorldState with the cube (static or carried), ring, local table patch, every box and the
    machine's wall/ceiling (workcell: live poses, see cycle.workcell_scene)."""
    s, size = settings, settings.cube_size_mm
    cube_top, goal_top = np.array(targets['cube_top_world_mm']), np.array(targets['goal_top_world_mm'])
    source = targets.get('source_place', 'table')
    # No padding while the cube is in, or passes through, a box opening (it is tight);
    # 10 mm around the cube otherwise.
    in_box = {'at_pick': source in s.boxes, 'placed': goal in s.boxes or goal in s.side_boxes,
              'carried': source in s.boxes or goal in s.boxes or goal in s.side_boxes}[cube_state]
    pad = 0.0 if in_box else 10.0
    obstacles = []
    # Tabletop patch around both ends; its top is the table (taught contact - cube size).
    low = np.minimum(cube_top[:2], goal_top[:2])-size*.7
    high = np.maximum(cube_top[:2], goal_top[:2])+size*.7
    table_top = s.calibration['pickup_tcp_z_mm']-size
    obstacles.append(prism(TABLE, [*(low+high)/2, table_top-100], [*(high-low), 200]))
    ring = observation.get('ring')
    top = s.calibration['place_tcp_z_mm']-size
    if s.ring_spot:
        xy, h, d = s.ring_spot['xy_mm'], s.ring_spot['height_mm'], s.ring_spot['diameter_mm']
        obstacles.extend(ring_prisms(xy, top-h/2, d, h))
    elif ring:
        from .calibration import calibrated_xy
        xy = calibrated_xy(s.calibration, ring['top_center_world_mm'])
        h, d = float(ring['height_mm']), float(ring['bounding_diameter_mm'])
        obstacles.extend(ring_prisms(xy, top-h/2, d, h))
    obstacles.extend(box_obstacles(s))
    obstacles.extend(other_cubes(s, observation))
    side = s.side_boxes.get(goal)
    obstacles.extend(workcell_prisms(workcell, s.carry_clearance_mm, side['panels'] if side else ()))
    if cube_state == 'carried':
        # Tool Z points down into the cube. The cup sits press_depth_mm inside the top
        # face, so the cube spans [-press, size] along tool Z (conservative both ways).
        contact = targets['contact']
        offset = np.array([contact['x']-cube_top[0], contact['y']-cube_top[1], 0.0])
        local = np.array(pose_matrix(contact))[:3, :3].T @ (-offset)
        press = s.press_depth_mm
        cube = prism(CUBE, [local[0], local[1], (size-press)/2], [size+pad, size+pad, size+press])
        observer = PoseInFrame(reference_frame=s.gripper, pose=Pose(o_z=1))
    else:
        top, yaw = (cube_top, targets['cube_yaw_deg']) if cube_state == 'at_pick' else (goal_top, targets['place_yaw_deg'])
        if cube_state == 'placed' and goal in s.side_boxes:  # lying in the side box: its gripped face looks at -x
            cube = prism(CUBE, [top[0]+size/2, top[1], top[2]], [size+pad, size+pad, size+pad])
        else:
            cube = prism(CUBE, [top[0], top[1], top[2]-size/2], [size+pad, size+pad, size], yaw)
        observer = PoseInFrame(reference_frame='world', pose=Pose(o_z=1))
    return WorldState(obstacles=[GeometriesInFrame(reference_frame='world', geometries=obstacles)],
                      transforms=[Transform(reference_frame=CUBE, pose_in_observer_frame=observer, physical_object=cube)])


# The ring is round. Four bars (length d, width d*sin 22.5 deg) at 0/45/90/135 deg cover the
# disc and stick out at most 7% beyond it; one d x d square adds 41% at its corners.
RING_BARS = [(RING if a == 0 else f'{RING}-{a}', a) for a in (0, 45, 90, 135)]


def ring_prisms(xy, z, d, h):
    w = d*np.sin(np.radians(22.5))
    return [prism(label, [*xy, z], [d, w, h], a) for label, a in RING_BARS]


def other_cubes(settings, observation):
    """Every visible cube except the selected one, resting on the calibrated tabletop."""
    from .calibration import calibrated_xy
    size, chosen = settings.cube_size_mm, observation.get('cube', {}).get('top_center_world_mm')
    result = []
    for i, c in enumerate(observation.get('cubes', [])):
        if chosen is not None and np.linalg.norm(np.subtract(c['top_center_world_mm'][:2], chosen[:2])) < 50:
            continue
        xy = calibrated_xy(settings.calibration, c['top_center_world_mm'])
        yaw = c['world_yaw_deg']+settings.calibration['yaw_correction_deg']
        top = settings.calibration['pickup_tcp_z_mm']
        result.append(prism(f'other-cube-{i}', [*xy, top-size/2], [size+10, size+10, size], yaw))
    return result


def box_obstacles(settings):
    return [prism(label, xyz, dims, yaw) for name, box in settings.boxes.items()
            for label, xyz, dims, yaw in cavity_geometry(box, name)]


def supports(settings, place):
    """Frames the cube rests on at a place (allowed cube contacts there)."""
    if place in settings.boxes:
        return [floor_label(place)]
    if place in settings.side_boxes:
        return ['scene-'+f for f in settings.side_boxes[place]['panels'] if 'floor' in f]
    if place == 'ring':
        return [label for label, *_ in RING_BARS]
    return ['table', TABLE]  # home or anywhere on the table


def allowed_contacts(settings, contacts, goal, source='table'):
    pairs = [(settings.gripper, CUBE)]
    if contacts == 'pickup':
        pairs += [(CUBE, f) for f in supports(settings, source)]
    elif contacts == 'place':
        pairs += [(CUBE, f) for f in supports(settings, goal)]
    return pairs


class PlanError(RuntimeError):
    pass


class Planner:
    def __init__(self, motion, settings, kinematics, workcell=()):
        self.motion, self.settings, self.kin, self.workcell = motion, settings, kinematics, list(workcell)

    async def segment(self, start_deg, world, goal_deg=None, pose=None, straight=False, allow=(), margin=None, upright=None):
        """upright: keep the tool orientation within this many degrees of the start->goal
        interpolation (a carried cube stays hanging under the cups instead of swinging out)."""
        s = self.settings
        constraints = Constraints(
            linear_constraint=[LinearConstraint(line_tolerance_mm=.5, orientation_tolerance_degs=.5)] if straight else [],
            orientation_constraint=[OrientationConstraint(orientation_tolerance_degs=upright)] if upright is not None and not straight else [],
            collision_specification=[CollisionSpecification(allows=[CollisionSpecification.AllowedFrameCollisions(frame1=a, frame2=b) for a, b in allow])])
        extra = {'start_state': {'configuration': {s.arm: [math.radians(v) for v in start_deg]}}, 'timeout': 30}
        if goal_deg is not None:
            extra['goal_state'] = {'configuration': {s.arm: [math.radians(v) for v in goal_deg]}}
        request = {'name': s.motion, 'component_name': s.gripper, 'extra': extra,
                   'world_state': MessageToDict(world, preserving_proto_field_name=True),
                   'constraints': MessageToDict(constraints, preserving_proto_field_name=True)}
        if pose is not None:
            request['destination'] = {'reference_frame': 'world', 'pose': pose}
        reply = await self.motion.do_command({'plan': json.dumps(request)}, timeout=180)
        return self.validate(reply, start_deg, goal_deg, pose, margin)

    def validate(self, reply, start_deg, goal_deg=None, pose=None, margin=None):
        s = self.settings
        margin = s.joint_limit_margin_deg if margin is None else margin
        if 'plan_partialwp' in reply:
            raise PlanError('Planner returned a partial path')
        nodes = reply.get('plan')
        if not isinstance(nodes, list) or len(nodes) < 2:
            raise PlanError('Planner returned no complete trajectory')
        # Every node keeps `margin` from each limit, except where a requested endpoint
        # (the current pose or a calibrated joint goal such as the overview) is already
        # closer; then the bound is that endpoint's own value.
        ends = [list(start_deg)]+([list(goal_deg)] if goal_deg is not None else [])
        bounds = [(min(lo+margin, *(e[i] for e in ends))-1e-6, max(hi-margin, *(e[i] for e in ends))+1e-6)
                  for i, (lo, hi) in enumerate(self.kin.limits_deg)]
        path = []
        for node in nodes:
            q = node.get(s.arm, [])
            if len(q) != 6 or not all(math.isfinite(v) for v in q):
                raise PlanError('Invalid planner joint output')
            for name, values in node.items():
                if name != s.arm and values != nodes[0].get(name):
                    raise PlanError('Planner tried to move another resource: '+name)
            q = [math.degrees(v) for v in q]
            if not all(lo <= v <= hi for v, (lo, hi) in zip(q, bounds)) or not self.kin.within_limits(q, .5):
                raise PlanError(f'Plan comes within {margin:g} deg of a joint limit: '+', '.join(f'{v:.1f}' for v in q))
            path.append(q)
        if max(abs(a-b) for a, b in zip(path[0], start_deg)) > .1:
            raise PlanError('Plan start does not match the requested start')
        if goal_deg is not None and max(abs(a-b) for a, b in zip(path[-1], goal_deg)) > .1:
            raise PlanError('Plan does not reach the joint goal')
        if pose is not None:
            T, G = self.kin.fk(path[-1]), np.array(pose_matrix(pose))
            angle = math.degrees(math.acos(np.clip((np.trace(T[:3, :3].T @ G[:3, :3])-1)/2, -1, 1)))
            if np.linalg.norm(T[:3, 3]-G[:3, 3]) > 1 or angle > .5:
                raise PlanError('Plan end does not reach the target tool pose')
        for a, b in zip(path, path[1:]):
            if pose is not None and max(abs(x-y) for x, y in zip(a, b)) > 10:
                raise PlanError('Straight-line plan jumps between IK branches')
        return path

    async def _plan_stage(self, name, start, observation, targets, goal):
        """One stage from `start` with the carried-cube checks (re-plans up to 5 times)."""
        s = self.settings
        _, kind, speed, cube_state, contacts = next(st for st in STAGES if st[0] == name)
        destinations = {'approach': ('joint', targets['q_above_pick']), 'descend': ('pose', targets['pre_contact']),
                        'press': ('pose', targets['press']), 'lift': ('pose', targets['above_pick']),
                        'transfer': ('joint', targets['q_above_place']), 'place': ('pose', targets['release']),
                        'retract': ('pose', targets['above_place']), 'return': ('joint', s.overview_joints_deg)}
        world = scene_world(s, observation, targets, cube_state, goal, self.workcell)
        what, value = destinations[name]
        try:
            side = s.side_boxes.get(goal)
            # Into a side box the cube is turned level on the way, so that carry has no upright rule;
            # the straight slide in is checked against everything but the box it enters.
            upright = s.carry_upright_deg if cube_state == 'carried' and kind == 'joint' and not side else None
            checked = [w for w in self.workcell if not (side and name in ('place', 'retract') and w[0] in side['panels'])]
            for attempt in range(5):
                path = await self.segment(start, world, goal_deg=value if what == 'joint' else None,
                                          pose=value if what == 'pose' else None, straight=kind == 'straight',
                                          allow=allowed_contacts(s, contacts, goal, targets.get('source_place', 'table')),
                                          upright=upright)
                # The carried cube is part of the arm: its swept volume must clear the walls
                # and ceiling with margin. The planner is sampling-based, so re-plan if not.
                bad = carried_cube_violations(self.kin, path, targets['grasp']['tcp_in_cube'], s.cube_size_mm,
                                              [w for w in checked if not (side and w[0] in side['panels'])], s.carry_clearance_mm/2) if cube_state == 'carried' else []
                if cube_state == 'carried' and side and name == 'transfer':
                    # The carry ends with the cube just outside this box's opening: only real contact counts.
                    bad += carried_cube_violations(self.kin, path, targets['grasp']['tcp_in_cube'], s.cube_size_mm,
                                                   [w for w in self.workcell if w[0] in side['panels']], 3.0)
                if cube_state == 'carried' and kind == 'joint' and upright is not None:
                    tilt = max_tool_tilt_deg(self.kin, path)
                    if tilt > s.carry_upright_deg+5:
                        bad = bad+[('tool tilt', -tilt)]
                if not bad:
                    return path
            raise PlanError('carried cube comes within '+f'{s.carry_clearance_mm/2:g} mm of '+', '.join(f'{n} (gap {g:.0f} mm)' for n, g in bad)
                            +' on every attempt; raise the ceiling or move the goal away from the wall')
        except Exception as e:
            hint = ''
            if 'violation between' in str(e):
                hint = (' -- a modelled obstacle blocks this stage. If it names a wall or a goal-<box>-wall '
                        'while the goal is the ring, move the ring clear of the boxes and walls.')
            raise PlanError(f'Planning {name} failed: {e}{hint}') from e

    async def plan_cycle(self, observation, targets, start_deg, goal):
        """Plan all stages; returns [(stage name, path_deg, speed_deg_s)] in execution order.

        Independent chains are planned concurrently: their joints are known in advance
        (q_above_pick / q_above_place are IK joint goals), so approach | descend-press-lift |
        transfer | place-retract | return need not wait for each other. A junction whose two sides
        do not meet exactly (a straight segment ends on a slightly different IK branch) is re-planned
        from the real end, so the executed path is always continuous."""
        s = self.settings
        speeds = {'travel': s.travel_speed_deg_s, 'approach': s.approach_speed_deg_s, 'press': s.press_speed_deg_s}
        qp, qq = list(targets['q_above_pick']), list(targets['q_above_place'])

        async def chain(names, start):
            out = []
            for name in names:
                path = await self._plan_stage(name, start, observation, targets, goal)
                out.append((name, path))
                start = path[-1]
            return out

        parts = await asyncio.gather(chain(['approach'], list(start_deg)), chain(['descend', 'press', 'lift'], qp),
                                     chain(['transfer'], qp), chain(['place', 'retract'], qq), chain(['return'], qq))
        paths = dict(item for part in parts for item in part)

        def meets(a, b):
            return max(abs(x-y) for x, y in zip(a, b)) <= .1
        if not meets(paths['lift'][-1], paths['transfer'][0]):
            paths['transfer'] = await self._plan_stage('transfer', paths['lift'][-1], observation, targets, goal)
        if not meets(paths['retract'][-1], paths['return'][0]):
            paths['return'] = await self._plan_stage('return', paths['retract'][-1], observation, targets, goal)
        return [(name, paths[name], speeds[speed]) for name, _, speed, _, _ in STAGES]

    async def plan_reset(self, start_deg, observation=None):
        """From any in-margin pose: straight retreat up if low, then a joint move to the overview."""
        s = self.settings
        if max(abs(a-b) for a, b in zip(start_deg, s.overview_joints_deg)) <= .25:
            return []
        if not self.kin.within_limits(start_deg, 1.0):
            raise PlanError('A joint is at its limit; run {"action":"recover_limits","workcell_ready":true} first')
        static = [prism('reset-objects-envelope', [400, 0, 125], [600, 800, 350])]
        box = box_obstacles(s)
        tcp = self.kin.fk(start_deg)[:3, 3]
        plans, start = [], list(start_deg)
        if tcp[2] < 330:
            from .transforms import matrix_pose
            up = matrix_pose(self.kin.fk(start_deg).tolist())
            up['z'] = max(float(tcp[2])+80, 360.0)
            world = WorldState(obstacles=[GeometriesInFrame(reference_frame='world', geometries=box)])
            path = await self.segment(start, world, pose=up, straight=True, margin=RESET_MARGIN_DEG)
            plans.append(('reset_retreat', path, s.approach_speed_deg_s))
            start = path[-1]
        world = WorldState(obstacles=[GeometriesInFrame(reference_frame='world', geometries=static+box)])
        path = await self.segment(start, world, goal_deg=s.overview_joints_deg, margin=RESET_MARGIN_DEG)
        plans.append(('reset_to_overview', path, s.travel_speed_deg_s))
        return plans
