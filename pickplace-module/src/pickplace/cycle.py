"""One pick-and-place cycle on the real arm.

Order: release suction -> return to the calibrated overview -> 5-frame RGB-D detection
-> plan every stage -> approach, descend, SUCTION ON, press press_depth_mm into the cube,
dwell, lift, transfer, place, SUCTION OFF, dwell, retract, return.

Suction is OPEN LOOP: the vacuum gripper's IsHoldingSomething feedback is not read
anywhere (its driver reports inconsistently). Alignment comes from vision; collision
avoidance and centring come from the motion planner.
"""
import asyncio
import copy
import time
import uuid
from datetime import datetime, timezone

import cv2
import numpy as np
from google.protobuf.json_format import MessageToDict
from viam.proto.component.arm import JointPositions, MoveOptions

from .kinematics import ArmKinematics
from .memory import CubeMemory
from .planning import Planner
from .task import compute_targets
from .transforms import pose_matrix
from .vision_geometry import analyze, consensus_observation, decode_depth, native


class CycleError(RuntimeError):
    pass


class PickPlaceCycle:
    def __init__(self, settings, arm, gripper, camera, motion, lease=None, log=None):
        self.s, self.arm, self.gripper, self.camera, self.motion, self.lease = settings, arm, gripper, camera, motion, lease
        self.log = log or (lambda message: None)
        self.stage = 'idle'
        self.events = []
        self.observation = {}
        self.targets = {}
        self.plan_summary = {}
        self.lease_token = ''
        self.kin = None
        self.last_frame = None
        self.memory = CubeMemory()

    # ---- bookkeeping -------------------------------------------------------
    def event(self, stage, **detail):
        self.stage = stage
        entry = {'t': datetime.now(timezone.utc).isoformat(timespec='milliseconds'), 'stage': stage, **detail}
        self.events.append(entry)
        self.events = self.events[-100:]
        self.log(f'{stage} {detail}' if detail else stage)

    async def read(self, call, attempts=3):
        """Retry a READ-ONLY robot call: the xArm driver occasionally fails single status reads
        (seen constantly while the e-stop is latched). Never used for motion commands."""
        for i in range(attempts):
            try:
                return await call()
            except asyncio.CancelledError:
                raise
            except Exception:
                if i == attempts-1:
                    raise
                await asyncio.sleep(.2*(i+1))

    async def pose(self, component, frame):
        return MessageToDict(await self.read(lambda: self.motion.get_pose(component, frame, timeout=5)))['pose']

    async def joints(self):
        return [float(v) for v in (await self.read(lambda: self.arm.get_joint_positions(timeout=3))).values]

    # ---- setup -------------------------------------------------------------
    async def load_kinematics(self):
        """Arm URDF + measured gripper TCP; verified against the live GetPose."""
        kind, raw = (await self.read(lambda: self.arm.get_kinematics(timeout=10)))[:2]
        if kind != 2:
            raise CycleError('URDF arm kinematics are required (Arm.GetKinematics returned another format)')
        tool = await self.pose(self.s.gripper, self.s.arm)
        T = np.array(pose_matrix(tool))
        if np.max(np.abs(T[:3, :3]-np.eye(3))) > 1e-3 or np.linalg.norm(T[:2, 3]) > .1:
            raise CycleError('Gripper frame must be a pure Z offset from the arm flange')
        kin = ArmKinematics(raw.decode() if isinstance(raw, bytes) else raw, tool_offset_mm=float(T[2, 3]))
        q = await self.joints()
        live = np.array(pose_matrix(await self.pose(self.s.gripper, 'world')))
        if np.linalg.norm(kin.fk(q)[:3, 3]-live[:3, 3]) > .5:
            raise CycleError('Local kinematics disagree with the frame system; is the arm still at the world origin?')
        self.kin = kin
        return kin

    async def preflight(self):
        if self.lease is not None:
            status = await self.lease.do_command({'action': 'status'}, timeout=5)
            if status.get('busy') or (status.get('reserved_by') and status.get('reserved_by') != self.lease_token):
                raise CycleError(f'{self.s.lease_service} is busy or holds the arm reservation')
        if await self.read(lambda: self.arm.is_moving(timeout=3)):
            raise CycleError('Arm is moving')
        if self.s.camera_mount_reference:
            actual = np.array(pose_matrix(await self.pose(self.s.camera, self.s.arm)))
            reference = np.array(pose_matrix(self.s.camera_mount_reference))
            if np.linalg.norm(actual[:3, 3]-reference[:3, 3]) > .5 or np.max(np.abs(actual[:3, :3]-reference[:3, :3])) > 1e-3:
                raise CycleError('Camera mount transform differs from the calibrated camera_mount_reference')
        await self.load_kinematics()

    async def acquire_lease(self):
        if self.lease is None:
            return
        token = str(uuid.uuid4())
        reply = await self.lease.do_command({'action': 'acquire_lease', 'token': token}, timeout=5)
        if reply.get('reserved') is not True:
            raise CycleError('Could not reserve the arm through '+self.s.lease_service)
        self.lease_token = token

    async def release_lease(self):
        if self.lease is not None and self.lease_token:
            try:
                await self.lease.do_command({'action': 'release_lease', 'token': self.lease_token}, timeout=5)
            finally:
                self.lease_token = ''

    # ---- suction (open loop) ---------------------------------------------
    async def suction_on(self):
        self.event('suction_on')
        if await self.gripper.grab(timeout=5) is False:
            raise CycleError('Vacuum gripper rejected the Grab command')

    async def suction_off(self, dwell=True):
        self.event('suction_off')
        await self.gripper.open(timeout=5)
        if dwell:
            await asyncio.sleep(self.s.release_dwell_s)

    # ---- perception --------------------------------------------------------
    async def capture(self, require_ring, select='only', require_cube=True):
        q0 = await self.joints()
        props = MessageToDict(await self.read(lambda: self.camera.get_properties(timeout=5)))
        pose = await self.pose(self.s.camera, 'world')
        images, meta = await self.camera.get_images(timeout=10)
        stamp = MessageToDict(meta).get('capturedAt')
        if not stamp or abs(time.time()-datetime.fromisoformat(stamp.replace('Z', '+00:00')).timestamp()) > 3:
            raise CycleError('Camera image is stale or missing its capture timestamp')
        color = next((i for i in images if i.mime_type in ('image/jpeg', 'image/png')), None)
        depth = next((i for i in images if i.mime_type == 'image/vnd.viam.dep'), None)
        if color is None or depth is None:
            raise CycleError('Camera must return aligned color and depth')
        bgr = cv2.imdecode(np.frombuffer(color.data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            raise CycleError('Could not decode the color image')
        if max(abs(a-b) for a, b in zip(q0, await self.joints())) > .05:
            raise CycleError('Arm moved during camera capture')
        attrs = {'cube_size_mm': self.s.cube_size_mm, 'table_z_mm': self.s.table_z_mm}
        depth_mm = decode_depth(depth.data)
        obs = await asyncio.to_thread(analyze, bgr, depth_mm, props, pose, attrs, require_ring, select, require_cube)
        obs.update(capture_time=stamp, observation_joints_deg=q0)
        self.last_frame = (bgr, depth_mm, props, pose)
        return obs

    async def survey_box(self, name, frames=3):
        """No motion. From the overview, measure an orange Post-it box -> candidate `boxes` entry.

        Three frames must agree (centre 3 mm, opening 3 mm, yaw 1 deg). The cube is not
        required in view; the tabletop must be.
        """
        from .box_survey import survey
        await self.preflight()
        if max(abs(a-b) for a, b in zip(await self.joints(), self.s.overview_joints_deg)) > .25:
            raise CycleError('Box survey uses the overview calibration; run {"action":"reset"} first')
        self.event('surveying_box', name=name)
        found = []
        for _ in range(frames):
            obs = await self.capture(require_ring=False)
            result = await asyncio.to_thread(survey, *self.last_frame, obs, self.s.calibration, name, self.s.cube_size_mm)
            if 'issues' in result:
                raise CycleError('; '.join(result['issues']))
            found.append(result['candidate'])
            await asyncio.sleep(.15)
        centers = np.array([c['floor_center_world_mm'] for c in found])
        dims = np.array([c['inner_dimensions_mm'] for c in found])
        yaws = np.array([c['world_yaw_deg'] for c in found])
        if np.ptp(centers, axis=0).max() > 3 or np.ptp(dims, axis=0).max() > 3 or np.ptp(yaws) > 1:
            raise CycleError('Box measurement unstable across frames; keep the camera and box still')
        candidate = dict(found[int(np.argmin(np.linalg.norm(centers-np.median(centers, axis=0), axis=1)))])
        candidate['inner_dimensions_mm'] = dims.min(axis=0).tolist()  # tightest opening seen
        self.event('box_candidate', name=name)
        return {'candidate': candidate, 'frames': found,
                'next_step': 'Check the opening with a tape measure, then add the candidate to the "boxes" attribute'}

    async def detect(self, goal, select='only'):
        """Five consecutive frames; four (incl. the newest two) must agree on the selected cube."""
        self.event('detecting', select=select)
        frames, last = [], ''
        for i in range(9):
            obs = await self.capture(require_ring=goal == 'ring', select=select)
            issues = [e for e in obs.get('issues', []) if not e.startswith('Depth/world tabletop differs')]
            if issues:
                raise CycleError('; '.join(issues))
            frames.append(obs)
            if len(frames) >= 5:
                try:
                    self.observation = consensus_observation(frames[-5:])
                    return self.observation
                except ValueError as e:
                    last = str(e)
            await asyncio.sleep(.15)
        raise CycleError(last or 'Could not confirm stable detections')

    # ---- motion ------------------------------------------------------------
    async def execute(self, name, path, speed):
        q = await self.joints()
        if max(abs(a-b) for a, b in zip(q, path[0])) > .5:
            raise CycleError(f'Arm is not at the planned start of {name}')
        self.event(name, nodes=len(path), speed_deg_s=speed)
        await self.arm.move_through_joint_positions(
            [JointPositions(values=list(p)) for p in path],
            options=MoveOptions(max_vel_degs_per_sec=speed, max_acc_degs_per_sec2=self.s.acceleration_deg_s2), timeout=180)
        q = await self.joints()
        if max(abs(a-b) for a, b in zip(q, path[-1])) > .5:
            raise CycleError(f'Arm did not reach the end of {name}')

    async def stop_arm(self):
        try:
            await asyncio.shield(self.arm.stop(timeout=3))
        except Exception as e:  # a controller fault can make Stop itself fail
            self.event('arm_stop_error', error=str(e))

    # ---- commands ----------------------------------------------------------
    async def verify_outcome(self, goal, remembered=None):
        """Vision stands in for the unreliable suction sensor. Back at the overview: is a cube still
        where we picked it from the table? Is one at the goal, if the goal is a table spot?
        Never raises; reports what it saw."""
        await asyncio.sleep(.5)
        try:
            obs = await self.capture(require_ring=False, select='nearest', require_cube=False)
        except Exception as e:
            return {'cube_left_pick_spot': None, 'detail': f'check failed: {e}'}
        cubes = [c['top_center_world_mm'] for c in obs.get('cubes', [])]
        result = {'cubes_visible': len(cubes)}
        picked = None if remembered else self.observation.get('cube', {}).get('top_center_world_mm')
        if picked is not None:
            still = any(np.linalg.norm(np.subtract(c[:2], picked[:2])) < 40 for c in cubes)
            result['cube_left_pick_spot'] = not still
        if goal == 'home':
            from .calibration import calibrated_xy
            result['cube_at_goal'] = any(np.linalg.norm(calibrated_xy(self.s.calibration, c)-self.s.home['xy_mm']) < 40 for c in cubes)
        if result.get('cube_left_pick_spot') is False:
            result['detail'] = 'a cube is still at the pick spot: suction likely did not hold'
        elif result.get('cube_at_goal') is False:
            result['detail'] = 'no cube seen at the home spot'
        elif result.get('cube_at_goal'):
            result['detail'] = 'cube seen at the home spot'
        else:
            result['detail'] = 'pick spot is empty' if picked is not None else 'not checked (cube was not on the table)'
        return result

    async def recover_limits(self):
        """Move ONLY joints sitting inside the limit margin (<= margin + 1 deg) back into range.

        Needed after a controller joint-limit fault (clear the fault first). The tool can
        shift by roughly 15 mm, so suction is switched off before moving.
        """
        await self.preflight()
        q = await self.joints()
        m = self.s.joint_limit_margin_deg
        # Only joints within 1 deg of (or past) a hard limit move, to the margin inside it.
        target = [v if lo+1 <= v <= hi-1 else min(max(v, lo+m), hi-m) for v, (lo, hi) in zip(q, self.kin.limits_deg)]
        moves = [abs(a-b) for a, b in zip(q, target)]
        if max(moves) == 0:
            return {'moved': False}
        if max(moves) > m+1:
            raise CycleError('A joint is too far outside its range for automatic recovery; jog it manually')
        await self.acquire_lease()
        try:
            # After a limit fault (and a released e-stop) the controller stays in error until
            # cleared; the viam:ufactory driver exposes that as an arm DoCommand.
            try:
                self.event('clear_error', reply=await self.arm.do_command({'clear_error': True}, timeout=10))
            except Exception as e:
                self.event('clear_error_failed', error=str(e))
            await self.suction_off()
            self.event('recover_limits', target=target)
            await self.arm.move_through_joint_positions(
                [JointPositions(values=target)],
                options=MoveOptions(max_vel_degs_per_sec=2, max_acc_degs_per_sec2=self.s.acceleration_deg_s2), timeout=60)
        except BaseException:
            await self.stop_arm()
            raise
        finally:
            await self.release_lease()
        return {'moved': True, 'from': q, 'to': target}

    async def go_to_overview(self, planner):
        await self.suction_off()
        self.event('planning_reset')
        for name, path, speed in await planner.plan_reset(await self.joints()):
            await self.execute(name, path, speed)
        self.event('settling_camera')
        await asyncio.sleep(1)

    async def plan(self, goal, observation=None, start=None, source=None):
        """Plan the full cycle from `start` (default: current joints). No motion."""
        planner = Planner(self.motion, self.s, self.kin)
        observation = observation or self.observation
        start = start or await self.joints()
        self.event('planning', goal=goal)
        self.targets = native(await asyncio.to_thread(compute_targets, self.s, observation, goal, self.kin, start, source))
        plans = await planner.plan_cycle(observation, self.targets, start, goal)
        self.plan_summary = {
            'goal': goal,
            'stages': {name: {'nodes': len(path), 'speed_deg_s': speed,
                              'min_limit_clearance_deg': round(min(self.kin.limit_clearance_deg(q) for q in path), 2)}
                       for name, path, speed in plans},
            'pickup_tcp_world': self.targets['press'], 'release_tcp_world': self.targets['release'],
            'press_depth_mm': self.s.press_depth_mm, 'cube_yaw_deg': self.targets['cube_yaw_deg'],
            'place_yaw_deg': self.targets['place_yaw_deg'], 'source_place': self.targets['source_place'],
        }
        return plans

    async def look_for_obstacles(self):
        """When the cube is not on the table: one frame for the ring and any other cubes."""
        try:
            obs = await self.capture(require_ring=False, require_cube=False)
        except Exception as e:
            self.event('scene_check_failed', error=str(e))
            return {}
        issues = [i for i in obs.get('issues', []) if not i.startswith('Depth/world tabletop')]
        if issues:
            self.event('scene_check_issues', issues=issues)
            return {k: obs[k] for k in ('camera_pose_world', 'camera_intrinsics', 'capture_time', 'observation_joints_deg') if k in obs}
        obs.pop('cube', None)
        return obs

    async def run(self, goal, select='only', source='auto'):
        """One full cycle to `goal`. The cube comes from vision when it is on the table, or from
        the remembered placement when it sits in a box or on the ring (source='auto').
        On failure the arm stops and suction stays as it was."""
        self.events = []
        touched = False
        await self.preflight()
        await self.acquire_lease()
        try:
            await self.go_to_overview(Planner(self.motion, self.s, self.kin))
            remembered = self.memory.pickable_source() if source == 'auto' else source
            if remembered is None:
                await self.detect(goal, select)
            else:
                self.event('using_remembered_cube', place=remembered['place'])
                self.observation = await self.look_for_obstacles()
                if goal == 'ring' and 'ring' not in self.observation:
                    raise CycleError('Ring goal needs the ring in view from the overview')
            if remembered is not None and remembered['place'] == goal:
                raise CycleError(f'The cube is already at {goal}')
            plans = dict((name, (path, speed)) for name, path, speed in await self.plan(goal, source=remembered))
            for name in ('approach', 'descend'):
                await self.execute(name, *plans[name])
            touched = True  # from here on a failure may leave the cube anywhere along the path
            await self.suction_on()
            await self.execute('press', *plans['press'])
            self.event('suction_dwell', seconds=self.s.suction_dwell_s)
            await asyncio.sleep(self.s.suction_dwell_s)
            for name in ('lift', 'transfer', 'place'):
                await self.execute(name, *plans[name])
            await self.suction_off()
            for name in ('retract', 'return'):
                await self.execute(name, *plans[name])
            outcome = await self.verify_outcome(goal, remembered)
            self.remember(goal, outcome)
            self.event('complete', goal=goal, **outcome)
            return {**copy.deepcopy(self.plan_summary), 'outcome': outcome, 'cube_location': copy.deepcopy(self.memory.location)}
        except BaseException:
            if touched:
                self.memory.set('unknown', how='cycle stopped or failed after suction on; check the workcell')
            await self.stop_arm()
            raise
        finally:
            await self.release_lease()

    def remember(self, goal, outcome):
        t = self.targets
        if outcome.get('cube_left_pick_spot') is False:
            # Suction did not hold: the cube is still where it was picked.
            if t['source_place'] in ('table', 'home'):
                self.memory.set(t['source_place'], how='vision: still at the pick spot after the cycle')
            return
        self.memory.set(goal, t['goal_top_world_mm'], t['place_yaw_deg'], t['grasp']['tcp_in_cube'],
                        how='placed by code-2' + (' and seen there' if outcome.get('cube_at_goal') else ''))

    async def reset_cube(self):
        """Put the cube back on its home table spot, from wherever code-2 last placed it."""
        if not self.s.home:
            raise CycleError('No home spot configured (attribute "home": {"xy_mm": [x, y], "yaw_deg": a})')
        place = self.memory.location.get('place')
        if place == 'home':
            return {'moved': False, 'detail': 'cube is already home', 'cube_location': copy.deepcopy(self.memory.location)}
        if place == 'unknown':
            raise CycleError('Cube location unknown: put it on the table in view (or set_cube_location), then retry')
        return await self.run('home')

    def set_location(self, place, goals):
        """Operator correction after moving the cube by hand. No motion."""
        if place not in ('unknown', 'table', *goals):
            raise CycleError('place must be unknown, table or one of: '+', '.join(goals))
        if place in ('unknown', 'table', 'home'):
            return self.memory.set(place, how='set by operator')
        # A box or the ring: assume it sits where code-2 would place it, gripped like a fresh pick.
        from .task import goal_surface
        top, yaw, _ = goal_surface(self.s, self.observation or {}, place) if place != 'ring' else (None, None, None)
        if top is None:
            raise CycleError('Cannot locate the ring without vision; use pick_place to put the cube there')
        from .grasp_geometry import describe_grasp
        from .vision_geometry import downward_pose
        size = self.s.cube_size_mm
        G = describe_grasp(top, downward_pose(top, yaw), yaw, [size, size])['tcp_in_cube']  # cup at the face centre
        return self.memory.set(place, top.tolist(), float(yaw), G, how='set by operator (assumed seated at the goal)')

    async def reset(self):
        await self.preflight()
        await self.acquire_lease()
        try:
            await self.go_to_overview(Planner(self.motion, self.s, self.kin))
            self.event('at_overview')
        except BaseException:
            await self.stop_arm()
            raise
        finally:
            await self.release_lease()

    async def self_test(self):
        """No motion and no gripper calls: checks everything a cycle depends on from the current pose."""
        report = {'checks': {}}
        checks = report['checks']

        async def check(name, fn):
            try:
                checks[name] = {'ok': True, **((await fn()) or {})}
            except Exception as e:
                checks[name] = {'ok': False, 'error': f'{type(e).__name__}: {e}'}

        async def kinematics():
            await self.preflight()
            q = await self.joints()
            return {'joints_deg': q, 'min_limit_clearance_deg': round(self.kin.limit_clearance_deg(q), 3),
                    'at_overview': max(abs(a-b) for a, b in zip(q, self.s.overview_joints_deg)) <= .25}

        async def camera():
            obs = await self.capture(require_ring=False)
            props = obs.get('camera_intrinsics')
            same = props == self.s.calibration.get('camera_intrinsics')
            return {'intrinsics_match_calibration': same, 'cube_count': obs.get('cube_count'),
                    'ring_count': obs.get('ring_count'), 'issues': obs.get('issues', [])}

        async def reset_plan():
            plans = await Planner(self.motion, self.s, self.kin).plan_reset(await self.joints())
            return {'stages': [(name, len(path)) for name, path, _ in plans]}

        self.event('self_test')
        await check('preflight_and_kinematics', kinematics)
        await check('camera', camera)
        if self.kin is not None:
            await check('reset_plan', reset_plan)
        report['ok'] = all(c['ok'] for c in checks.values())
        report['goals'] = self.s.goals
        self.event('self_test_done', ok=report['ok'])
        return report

    async def dry_run(self, goal, select='only'):
        """Detect and plan at the overview without moving anything."""
        await self.preflight()
        q = await self.joints()
        if max(abs(a-b) for a, b in zip(q, self.s.overview_joints_deg)) > .25:
            raise CycleError('Dry run detects from the calibrated overview; run {"action":"reset"} first')
        remembered = self.memory.pickable_source()
        if remembered is None:
            await self.detect(goal, select)
        else:
            self.event('using_remembered_cube', place=remembered['place'])
            self.observation = await self.look_for_obstacles()
        await self.plan(goal, source=remembered)
        self.event('planned', goal=goal)
        return copy.deepcopy(self.plan_summary)

