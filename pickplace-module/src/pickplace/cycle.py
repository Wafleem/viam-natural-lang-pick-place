"""One pick-and-place cycle on the real arm.

Order: release suction -> return to the calibrated overview -> 5-frame RGB-D detection
-> plan every stage -> approach, descend, SUCTION ON, press press_depth_mm into the cube,
dwell, lift, transfer, place, SUCTION OFF, dwell, retract, return.

Suction is CLOSED LOOP since the driver update: the press waits for IsHoldingSomething to
report a seal, the lift is checked with it, and a missed seal or a drop triggers a re-pick
(pick_retries). Alignment comes from vision; collision avoidance and centring come from
the motion planner.
"""
import asyncio
import math
import copy
import json
import time
import uuid
from datetime import datetime, timezone

import cv2
import numpy as np
from google.protobuf.json_format import MessageToDict
from viam.proto.component.arm import JointPositions, MoveOptions

from .kinematics import ArmKinematics
from .memory import CubeMemory
from .planning import Planner, allowed_contacts, carried_cube_violations, scene_world
from .task import compute_targets
from .transforms import matrix_pose, pose_matrix
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

    async def clear_controller(self):
        """A collision trip latches the xArm controller; clearing it never moves the arm."""
        try:
            await self.arm.do_command({'clear_error': True}, timeout=10)
        except Exception as e:
            self.event('clear_error_failed', error=str(e)[:120])

    async def acquire_lease(self):
        await self.clear_controller()
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
        # Open loop: Grab's bool mirrors the unreliable holding sensor (false in the air), so it
        # never gates the press. Only an exception means the command itself failed.
        reply = await self.gripper.grab(timeout=5)
        if reply is False:
            self.event('grab_reported_not_holding', note='ignored: open-loop suction')

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
        geometry_pose = pose
        corr = self.s.camera_correction
        if corr:  # replacement camera: rescale its depth and map its frame onto the calibrated one
            depth_mm = depth_mm*corr['depth_scale']
            geometry_pose = matrix_pose((np.array(pose_matrix(pose)) @ np.array(corr['matrix'])).tolist())
        obs = await asyncio.to_thread(analyze, bgr, depth_mm, props, geometry_pose, attrs, require_ring, select, require_cube)
        # The view check compares the arm's camera frame with the calibration's, so keep the raw one.
        obs.update(capture_time=stamp, observation_joints_deg=q0, camera_pose_world=pose)
        self.last_frame = (bgr, depth_mm, props, geometry_pose)
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

    async def detect(self, goal, select='only', allow_sweep=True):
        """Find the cube: the overview first, then a camera sweep if it is not fully in view."""
        try:
            return await self.detect_at_overview(goal, select)
        except CycleError as e:
            text = str(e)
            if not any(k in text for k in ('No cube in view', 'clipped', 'Cube top surface', 'Need exactly one full cube',
                                         'Surface tilt', 'No horizontal surface', 'Detected cube top dimensions', 'Cube height does not match')):
                raise
            self.event('cube_not_in_overview', reason=text[:120])
            if not allow_sweep:
                raise CycleError(text+' -- a camera sweep would move the arm: send dry_run with "workcell_ready": true')
        return await self.sweep_for_cube(goal, select)

    def overview_visible(self, xy, z=None, margin=25):
        """Is a tool-XY point (at cube-top height) inside the overview image?"""
        cal = self.s.calibration
        if not cal.get('camera_pose_world') or not cal.get('camera_intrinsics'):
            return None
        I, cam = cal['camera_intrinsics'], np.array(pose_matrix(cal['camera_pose_world']))
        a = np.radians(cal['yaw_correction_deg'])
        raw = np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]]) @ (np.asarray(xy[:2])-np.asarray(cal['xy_offset_mm']))
        c = np.linalg.inv(cam) @ np.r_[raw, cal['observed_cube_z_mm'] if z is None else z, 1]
        if c[2] <= 0:
            return False
        u, v = I['focalXPx']*c[0]/c[2]+I['centerXPx'], I['focalYPx']*c[1]/c[2]+I['centerYPx']
        return margin < u < I['widthPx']-margin and margin < v < I['heightPx']-margin

    async def sweep_for_cube(self, goal, select='only'):
        """Pan the camera (base joint) to nearby views until the cube is fully seen.

        A sweep view is the overview camera translated sideways (same orientation and height),
        so the overview calibration applies in camera terms: tool = cal(raw - shift) + shift.
        The arm returns to the overview before planning; the observation is expressed as if seen
        from there, so the rest of the cycle is unchanged.
        """
        from .calibration import calibrated_xy
        from .planning import Planner, box_obstacles, prism
        from viam.proto.common import GeometriesInFrame, WorldState
        cal, s = self.s.calibration, self.s
        ref = await self.capture(require_ring=False, require_cube=False)
        planner = Planner(self.motion, s, self.kin)
        world = WorldState(obstacles=[GeometriesInFrame(reference_frame='world', geometries=[
            prism('scan-objects-envelope', [400, 0, 125], [600, 800, 350])]+box_obstacles(s))])
        g = cal['yaw_correction_deg']
        a = np.radians(g)
        to_raw = np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])
        found, tried = None, []
        try:
            T0 = self.kin.fk(s.overview_joints_deg)
            for off in s.sweep_offsets_mm:
                T = T0.copy()
                T[0, 3] += off[0]
                T[1, 3] += off[1]
                try:  # same camera orientation and height, shifted sideways
                    q = self.kin.ik(T, s.overview_joints_deg, 3.0)
                except ValueError:
                    tried.append({'shift_mm': off, 'skipped': 'unreachable'})
                    continue
                path = await planner.segment(await self.joints(), world, goal_deg=q, margin=1.0)
                await self.execute(f'sweep_{off[0]:+g}_{off[1]:+g}', path, s.sweep_speed_deg_s)
                await asyncio.sleep(.6)
                hits = []
                cam_shift = np.array(pose_matrix(await self.pose(s.camera, 'world')))[:2, 3]-np.array(pose_matrix(cal['camera_pose_world']))[:2, 3]
                # A sweep view is the overview camera translated (same orientation), so a point seen
                # here maps to tool XY like the same pixel/depth at the overview, plus the shift:
                # tool = cal(raw - shift) + shift = calibrated_xy(raw) + (I - R) shift.
                # (The old ring anchor measured its correction at ring height and applied it
                # 170 mm higher at the cube top; with a ~20 deg camera tilt that was ~20 mm off.)
                corr = cam_shift-to_raw.T @ cam_shift
                for _ in range(4):
                    obs = await self.capture(require_ring=False, select=select, require_cube=False)
                    issues = [i for i in obs.get('issues', []) if not i.startswith('Depth/world tabletop')]
                    if issues or 'cube' not in obs:
                        continue
                    cube_z = obs['cube']['top_center_world_mm'][2]  # same camera height as the overview
                    if abs(cube_z-cal['observed_cube_z_mm']) > 12:
                        self.event('sweep_frame_rejected', reason=f'cube top {cube_z-cal["observed_cube_z_mm"]:+.0f} mm from the calibrated table height')
                        continue
                    hits.append((calibrated_xy(cal, obs['cube']['top_center_world_mm'])+corr, obs['cube']['world_yaw_deg']+g, obs['cube'], corr, cube_z))
                    await asyncio.sleep(.1)
                tried.append({'shift_mm': off, 'frames_with_cube': len(hits)})
                if len(hits) >= 3:
                    xy = np.median([h[0] for h in hits], axis=0)
                    yaws = np.array([h[1] for h in hits])
                    ref_yaw = yaws[0]
                    yaw = float(np.median(ref_yaw+(yaws-ref_yaw+45) % 90-45))
                    if max(np.linalg.norm(h[0]-xy) for h in hits) <= 5:
                        found = (xy, yaw, hits[-1][2], np.median([h[3] for h in hits], axis=0), off, float(np.median([h[4] for h in hits])))
                        break
            if found is None:
                raise CycleError('Cube not found in the overview or the sweep views: '+json.dumps(tried))
        finally:
            back = await planner.segment(await self.joints(), world, goal_deg=s.overview_joints_deg, margin=1.0)
            if len(back) > 1 and max(abs(x-y) for x, y in zip(back[0], back[-1])) > .1:
                await self.execute('sweep_return', back, s.sweep_speed_deg_s)
            await asyncio.sleep(.6)
        xy, yaw, cube, corr, off, cube_z = found
        obs = copy.deepcopy(ref)
        obs['camera_pose_world'], obs['camera_intrinsics'] = cal['camera_pose_world'], cal['camera_intrinsics']
        obs['observation_joints_deg'] = list(s.overview_joints_deg)
        raw = to_raw @ (xy-np.asarray(cal['xy_offset_mm']))
        cube = {**cube, 'top_center_world_mm': [float(raw[0]), float(raw[1]), cube_z],
                'world_yaw_deg': float((yaw-g+45) % 90-45)}
        obs['cube'], obs['cubes'] = cube, [cube]
        obs['sweep'] = {'shift_mm': off, 'shift_correction_mm': [float(c) for c in corr], 'cube_tool_xy_mm': [float(v) for v in xy]}
        self.observation = native(obs)
        self.event('cube_found_by_sweep', shift_mm=off, xy=[round(float(v), 1) for v in xy])
        return self.observation

    async def detect_at_overview(self, goal, select='only'):
        """Five consecutive frames; four (incl. the newest two) must agree on the selected cube."""
        self.event('detecting', select=select)
        frames, last = [], ''
        for i in range(9):
            obs = await self.capture(require_ring=goal == 'ring' and not self.s.ring_spot, select=select)
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
        if goal == 'home' and self.overview_visible(self.s.home['xy_mm']):
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

    async def workcell_scene(self):
        """Live world poses of the machine's wall/ceiling frames -> [(frame, xyz, dims, yaw)]."""
        scene = []
        for o in self.s.workcell_obstacles:
            try:
                p = await self.pose(o['frame'], 'world')
            except Exception as e:
                raise CycleError(f'Scene obstacle {o["frame"]} not found in the frame system: {e}') from e
            T = np.array(pose_matrix(p))
            scene.append((o['frame'], T[:3, 3].tolist(), o['dims_mm'], math.degrees(math.atan2(T[1, 0], T[0, 0]))))
        return scene

    async def plan(self, goal, observation=None, start=None, source=None):
        """Plan the full cycle from `start` (default: current joints). No motion."""
        self.workcell = await self.workcell_scene()
        planner = Planner(self.motion, self.s, self.kin, self.workcell)
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

    async def holding(self, reads=3):
        """Closed loop: the vacuum gripper's holding sensor (reliable since the driver update).
        True only if every one of `reads` quick reads agrees, so one glitch can't fake a grip."""
        for _ in range(reads):
            status = await self.read(lambda: self.gripper.is_holding_something(timeout=3))
            if not status.is_holding_something:
                return False
            await asyncio.sleep(.1)
        return True

    async def wait_for_seal(self):
        """At the bottom of the press: wait (up to suction_dwell_s) until the sensor reports a seal,
        then hold seal_settle_s more so it firms up. Returns the seconds it took, or None."""
        start = time.monotonic()
        while time.monotonic()-start < self.s.suction_dwell_s:
            if await self.holding(reads=1):
                took = time.monotonic()-start
                await asyncio.sleep(self.s.seal_settle_s)
                return round(took, 2) if await self.holding() else None
            await asyncio.sleep(.1)
        return None

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
                if goal == 'ring' and not self.s.ring_spot and 'ring' not in self.observation:
                    raise CycleError('Ring goal needs the ring in view from the overview')
            if remembered is not None and remembered['place'] == goal:
                raise CycleError(f'The cube is already at {goal}')
            for attempt in range(1, self.s.pick_retries+2):
                plans = dict((name, (path, speed)) for name, path, speed in await self.plan(goal, source=remembered))
                for name in ('approach', 'descend'):
                    await self.execute(name, *plans[name])
                touched = True  # from here on a failure may leave the cube anywhere along the path
                await self.suction_on()
                await self.execute('press', *plans['press'])
                sealed = await self.wait_for_seal()
                self.event('seal', attempt=attempt, sealed=sealed is not None, seconds=sealed)
                held = False
                if sealed is not None:
                    await self.execute('lift', *plans['lift'])
                    held = await self.holding()
                    self.event('hold_check', attempt=attempt, held=held)
                if held:
                    break
                if attempt > self.s.pick_retries:
                    raise CycleError(f'No grip (seal or lift) on all {attempt} attempts')
                self.event('retrying_pick', attempt=attempt+1)
                await self.go_to_overview(Planner(self.motion, self.s, self.kin))  # suction off first
                if remembered is None or remembered['place'] in ('table', 'home'):
                    remembered = None
                    await self.detect(goal, select)  # find it again (it may have fallen; sweeps if needed)
                # from a box/ring it fell back into the same place: re-pick from memory
            for name in ('transfer', 'place'):
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

    async def _carried_segment(self, planner, start, world, **kw):
        """A carried-cube segment that also passes the swept-cube wall/ceiling check."""
        for _ in range(5):
            path = await planner.segment(start, world, **kw)
            if not carried_cube_violations(self.kin, path, self.targets['grasp']['tcp_in_cube'], self.s.cube_size_mm,
                                           self.workcell, self.s.carry_clearance_mm/2):
                return path
        raise CycleError('Carried cube would come too close to the wall/ceiling on every plan')

    async def hover_trial(self, goal, select='only', seconds=30.0):
        """Placement check without placing. Pick the cube, carry it to above the goal (its bottom
        clears the rim by box_entry_clearance_mm, or clearance_mm over the ring/table), hold there
        `seconds` so a person can judge the offset, then carry it back and set it down exactly
        where it was picked. Cube memory is unchanged when this completes."""
        self.events = []
        touched = False
        await self.preflight()
        await self.acquire_lease()
        try:
            await self.go_to_overview(Planner(self.motion, self.s, self.kin))
            remembered = self.memory.pickable_source()
            if remembered is None:
                await self.detect(goal, select)
            else:
                self.event('using_remembered_cube', place=remembered['place'])
                self.observation = await self.look_for_obstacles()
            if remembered is not None and remembered['place'] == goal:
                raise CycleError(f'The cube is already at {goal}')
            plans = dict((name, (path, speed)) for name, path, speed in await self.plan(goal, source=remembered))
            planner = Planner(self.motion, self.s, self.kin, self.workcell)
            for name in ('approach', 'descend'):
                await self.execute(name, *plans[name])
            touched = True
            await self.suction_on()
            await self.execute('press', *plans['press'])
            sealed = await self.wait_for_seal()
            self.event('seal', attempt=1, sealed=sealed is not None, seconds=sealed)
            if sealed is None:
                await self.go_to_overview(planner)
                touched = False  # nothing was lifted
                raise CycleError('No seal on the cube; hover trial stopped before lifting')
            await self.execute('lift', *plans['lift'])
            if not await self.holding():
                raise CycleError('The holding sensor lost the cube at the lift')
            await self.execute('transfer', *plans['transfer'])
            t = self.targets
            info = {'goal': goal, 'release_target_xy_mm': [round(t['release']['x'], 1), round(t['release']['y'], 1)],
                    'hover_tcp_z_mm': round(t['above_place']['z'], 1), 'place_yaw_deg': t['place_yaw_deg'], 'seconds': seconds}
            self.event('hovering', **info)
            await asyncio.sleep(seconds)
            info['still_held'] = await self.holding()
            self.event('hover_done', held=info['still_held'])
            if not info['still_held']:
                raise CycleError('The cube dropped while hovering')
            carried = scene_world(self.s, self.observation, t, 'carried', goal, self.workcell)
            back = await self._carried_segment(planner, await self.joints(), carried, goal_deg=t['q_above_pick'], upright=self.s.carry_upright_deg)
            await self.execute('return_to_pick', back, self.s.travel_speed_deg_s)
            down = await self._carried_segment(planner, await self.joints(), carried, pose=t['contact'], straight=True,
                                               allow=allowed_contacts(self.s, 'pickup', goal, t['source_place']))
            await self.execute('set_down', down, self.s.approach_speed_deg_s)
            await self.suction_off()
            touched = False
            at_pick = scene_world(self.s, self.observation, t, 'at_pick', goal, self.workcell)
            up = await planner.segment(await self.joints(), at_pick, pose=t['above_pick'], straight=True,
                                       allow=allowed_contacts(self.s, 'pickup', goal, t['source_place']))
            await self.execute('retract', up, self.s.approach_speed_deg_s)
            await self.go_to_overview(planner)
            self.event('complete', goal=goal, hover_trial=True)
            return {'hover': info, 'seal_seconds': sealed, 'cube_location': copy.deepcopy(self.memory.location)}
        except BaseException:
            if touched:
                self.memory.set('unknown', how='hover trial stopped with the cube gripped; check the workcell')
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

    async def scan(self, select='only'):
        """Camera only: go to the overview, find the ring, and find the cube (sweeping if needed)."""
        self.events = []
        await self.preflight()
        await self.acquire_lease()
        try:
            await self.go_to_overview(Planner(self.motion, self.s, self.kin))
            obs = await self.detect('box', select)
            from .calibration import calibrated_xy
            cube = obs.get('sweep', {}).get('cube_tool_xy_mm') or calibrated_xy(self.s.calibration, obs['cube']['top_center_world_mm']).tolist()
            out = {'cube_xy_mm': [round(float(v), 1) for v in cube],
                   'cube_yaw_deg': round(float(obs['cube']['world_yaw_deg']+self.s.calibration['yaw_correction_deg']), 1),
                   'found_by': 'sweep' if 'sweep' in obs else 'overview', 'sweep': obs.get('sweep')}
            if 'ring' in obs:
                out['ring_xy_mm'] = [round(float(v), 1) for v in calibrated_xy(self.s.calibration, obs['ring']['top_center_world_mm'])]
            if self.memory.location.get('place') in ('unknown', 'table'):
                self.memory.set('table', how='seen by scan')
            self.event('scan_done', **{k: v for k, v in out.items() if k != 'sweep'})
            return out
        except BaseException:
            await self.stop_arm()
            raise
        finally:
            await self.release_lease()

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
        top, yaw, _ = goal_surface(self.s, self.observation or {}, place) if place != 'ring' or self.s.ring_spot else (None, None, None)
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

    async def dry_run(self, goal, select='only', allow_sweep=False):
        """Detect and plan at the overview without moving anything."""
        await self.preflight()
        q = await self.joints()
        if max(abs(a-b) for a, b in zip(q, self.s.overview_joints_deg)) > .25:
            raise CycleError('Dry run detects from the calibrated overview; run {"action":"reset"} first')
        remembered = self.memory.pickable_source()
        if remembered is None:
            if allow_sweep:
                await self.acquire_lease()
            try:
                await self.detect(goal, select, allow_sweep)
            finally:
                await self.release_lease()
        else:
            self.event('using_remembered_cube', place=remembered['place'])
            self.observation = await self.look_for_obstacles()
        await self.plan(goal, source=remembered)
        self.event('planned', goal=goal)
        return copy.deepcopy(self.plan_summary)

