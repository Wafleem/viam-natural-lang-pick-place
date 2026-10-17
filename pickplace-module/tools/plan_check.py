"""Plan a full cycle on the LIVE motion planner without moving anything.

Uses the machine's real frame system (arm URDF, gripper, camera, table/wall/ceiling
obstacles) plus this module's scene geometry. The start state is the calibrated overview
(or --start-current) and the observation is a saved overview frame (or --live-detect when
the arm is parked at the overview). Only the motion service's "plan" DoCommand is sent;
the arm, gripper and camera receive read-only calls.

    .venv/bin/python tools/plan_check.py --goal box
    .venv/bin/python tools/plan_check.py --goal ring --frame 1
"""
import argparse
import asyncio
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'tests'), str(ROOT/'tools')]

from connect import connect  # noqa: E402
from viam.components.arm import Arm  # noqa: E402
from viam.components.camera import Camera  # noqa: E402
from viam.services.motion import MotionClient  # noqa: E402

from pickplace.config import Settings  # noqa: E402
from pickplace.cycle import PickPlaceCycle  # noqa: E402
from pickplace.vision_geometry import analyze  # noqa: E402


class PlanOnly:
    """Motion wrapper that refuses everything except planning and pose reads."""

    def __init__(self, motion):
        self.motion = motion

    async def do_command(self, command, **kwargs):
        if set(command) != {'plan'}:
            raise RuntimeError('plan_check only sends plan requests')
        return await self.motion.do_command(command, **kwargs)

    async def get_pose(self, *args, **kwargs):
        return await self.motion.get_pose(*args, **kwargs)


class ReadOnlyArm:
    def __init__(self, arm):
        self.arm = arm

    async def get_joint_positions(self, **kwargs):
        return await self.arm.get_joint_positions(**kwargs)

    async def get_kinematics(self, **kwargs):
        return await self.arm.get_kinematics(**kwargs)

    async def is_moving(self, **kwargs):
        return await self.arm.is_moving(**kwargs)


def saved_observation(attrs, frame, require_ring):
    from conftest import load_frame
    meta, bgr, depth = load_frame(frame)
    obs = analyze(bgr, depth, meta['properties'], meta['camera_pose_world'], attrs, require_ring)
    obs.update(capture_time=meta['capture_time'], observation_joints_deg=meta['observation_joints_deg'])
    return obs


def move_ring(observation, calibration, target_xy):
    """What-if: put the observed ring so its calibrated (tool) XY becomes target_xy."""
    import numpy as np
    from pickplace.calibration import calibrated_xy
    if 'ring' not in observation:
        raise SystemExit('No ring in this observation')
    a = np.radians(calibration['yaw_correction_deg'])
    R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    delta = R.T @ (np.array(target_xy)-calibrated_xy(calibration, observation['ring']['top_center_world_mm']))
    observation['ring']['top_center_world_mm'][:2] = (np.array(observation['ring']['top_center_world_mm'][:2])+delta).tolist()


async def main(args):
    attrs = json.loads(Path(args.config).read_text())
    s = Settings.from_attributes(attrs)
    async with connect() as robot:
        arm = ReadOnlyArm(Arm.from_robot(robot, s.arm))
        motion = PlanOnly(MotionClient.from_robot(robot, s.motion))
        camera = Camera.from_robot(robot, s.camera)
        cycle = PickPlaceCycle(s, arm, None, camera, motion, None, log=print)
        kin = await cycle.load_kinematics()
        current = await cycle.joints()
        print('Live joints:', [round(v, 3) for v in current], 'min limit clearance', round(kin.limit_clearance_deg(current), 3))
        if args.live_detect:
            observation = await cycle.detect(args.goal)
        else:
            observation = saved_observation(attrs, args.frame, args.goal == 'ring')
        if args.ring_world_xy:
            move_ring(observation, s.calibration, args.ring_world_xy)
        start = current if args.start_current else s.overview_joints_deg
        source = None
        if args.from_box:
            # The cube exactly as a cycle into that box would leave it (same grip), then plan from there.
            from pickplace.task import compute_targets
            placed = compute_targets(s, observation, args.from_box, kin, start)
            source = {'place': args.from_box, 'top_world_mm': placed['goal_top_world_mm'], 'yaw_deg': placed['place_yaw_deg'],
                      'tcp_in_cube': placed['grasp']['tcp_in_cube']}
            observation = {k: v for k, v in observation.items() if k not in ('cube', 'cubes')}
        plans = await cycle.plan(args.goal, observation, start, source)
        report = {'goal': args.goal, 'start': start, 'summary': cycle.plan_summary,
                  'q_above_pick': cycle.targets['q_above_pick'], 'q_above_place': cycle.targets['q_above_place'],
                  'max_abs_j4_deg': max(abs(q[3]) for _, path, _ in plans for q in path)}
        from pickplace.planning import Planner
        try:
            reset = await Planner(motion, s, kin).plan_reset(current)
            report['reset_from_current'] = [(name, len(path)) for name, path, _ in reset]
        except Exception as e:
            report['reset_from_current'] = 'blocked: '+str(e)
        print(json.dumps(report, indent=1, default=float))
        if args.out:
            Path(args.out).write_text(json.dumps({**report, 'plans': plans}, indent=1, default=float))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--goal', default='box', help='ring or a configured box name')
    p.add_argument('--config', default=str(ROOT/'config'/'code2-attributes.json'))
    p.add_argument('--frame', type=int, default=0, help='saved overview frame (tests/data/overview_N)')
    p.add_argument('--live-detect', action='store_true', help='detect live (arm must be at the overview)')
    p.add_argument('--start-current', action='store_true', help='plan from the live joints instead of the overview')
    p.add_argument('--ring-world-xy', type=float, nargs=2, metavar=('X', 'Y'), help='what-if ring position (tool XY, mm)')
    p.add_argument('--from-box', metavar='BOX', help='plan picking the cube out of this box (as reset_cube does)')
    p.add_argument('--out', help='write the full plan JSON here')
    asyncio.run(main(p.parse_args()))
