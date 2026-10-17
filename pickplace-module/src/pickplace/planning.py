"""Collision-checked planning through the Viam builtin motion service (plan only).

Every stage is planned before anything moves. The motion service's "plan" DoCommand
never moves the arm; execution happens separately, stage by stage, in cycle.py.
Free-space moves use IK joint goals chosen in task.py; vertical moves use straight-line
pose goals. Every returned joint node must keep joint_limit_margin_deg from its limit.
"""
import json
import math

import numpy as np
from google.protobuf.json_format import MessageToDict
from viam.proto.common import Geometry, GeometriesInFrame, Pose, PoseInFrame, RectangularPrism, Transform, Vector3, WorldState
from viam.proto.service.motion import CollisionSpecification, Constraints, LinearConstraint

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


def scene_world(settings, observation, targets, cube_state, goal):
    """WorldState with the cube (static or carried), ring, local table patch and every box."""
    s, size = settings, settings.cube_size_mm
    cube_top, goal_top = np.array(targets['cube_top_world_mm']), np.array(targets['goal_top_world_mm'])
    # No padding into a box (the measured opening is tight); 10 mm around the cube otherwise.
    pad = 0.0 if goal in s.boxes else 10.0
    obstacles = []
    # Support patch whose top is the cube's bottom at pickup (taught contact - cube size).
    low = np.minimum(cube_top[:2], goal_top[:2])-size*.7
    high = np.maximum(cube_top[:2], goal_top[:2])+size*.7
    table_top = cube_top[2]-size
    obstacles.append(prism(TABLE, [*(low+high)/2, table_top-100], [*(high-low), 200]))
    ring = observation.get('ring')
    if ring:
        from .calibration import calibrated_xy
        xy = calibrated_xy(s.calibration, ring['top_center_world_mm'])
        top = s.calibration['place_tcp_z_mm']-size
        h, d = float(ring['height_mm']), float(ring['bounding_diameter_mm'])
        obstacles.append(prism(RING, [*xy, top-h/2], [d, d, h]))
    obstacles.extend(box_obstacles(s))
    obstacles.extend(other_cubes(s, observation))
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
        cube = prism(CUBE, [top[0], top[1], top[2]-size/2], [size+pad, size+pad, size], yaw)
        observer = PoseInFrame(reference_frame='world', pose=Pose(o_z=1))
    return WorldState(obstacles=[GeometriesInFrame(reference_frame='world', geometries=obstacles)],
                      transforms=[Transform(reference_frame=CUBE, pose_in_observer_frame=observer, physical_object=cube)])


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


def allowed_contacts(settings, contacts, goal):
    pairs = [(settings.gripper, CUBE)]
    if contacts == 'pickup':
        pairs += [(CUBE, 'table'), (CUBE, TABLE)]
    elif contacts == 'place':
        pairs += [(CUBE, floor_label(goal) if goal in settings.boxes else RING)]
    return pairs


class PlanError(RuntimeError):
    pass


class Planner:
    def __init__(self, motion, settings, kinematics):
        self.motion, self.settings, self.kin = motion, settings, kinematics

    async def segment(self, start_deg, world, goal_deg=None, pose=None, straight=False, allow=(), margin=None):
        s = self.settings
        constraints = Constraints(
            linear_constraint=[LinearConstraint(line_tolerance_mm=.5, orientation_tolerance_degs=.5)] if straight else [],
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

    async def plan_cycle(self, observation, targets, start_deg, goal):
        """Plan all stages in order; returns [(stage name, path_deg, speed_deg_s)]."""
        s = self.settings
        speeds = {'travel': s.travel_speed_deg_s, 'approach': s.approach_speed_deg_s, 'press': s.press_speed_deg_s}
        destinations = {'approach': ('joint', targets['q_above_pick']), 'descend': ('pose', targets['pre_contact']),
                        'press': ('pose', targets['press']), 'lift': ('pose', targets['above_pick']),
                        'transfer': ('joint', targets['q_above_place']), 'place': ('pose', targets['release']),
                        'retract': ('pose', targets['above_place']), 'return': ('joint', s.overview_joints_deg)}
        plans, start = [], list(start_deg)
        for name, kind, speed, cube_state, contacts in STAGES:
            world = scene_world(s, observation, targets, cube_state, goal)
            what, value = destinations[name]
            try:
                path = await self.segment(start, world, goal_deg=value if what == 'joint' else None,
                                          pose=value if what == 'pose' else None, straight=kind == 'straight',
                                          allow=allowed_contacts(s, contacts, goal))
            except Exception as e:
                hint = ''
                if 'violation between' in str(e):
                    hint = (' -- a modelled obstacle blocks this stage. If it names a wall or a goal-<box>-wall '
                            'while the goal is the ring, move the ring clear of the boxes and walls.')
                raise PlanError(f'Planning {name} failed: {e}{hint}') from e
            plans.append((name, path, speeds[speed]))
            start = path[-1]
        return plans

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
