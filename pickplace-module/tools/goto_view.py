"""Move the camera to a named view (camera-only, collision-checked, slow) and optionally save a frame.

    .venv/bin/python tools/goto_view.py box_view --save /tmp/boxview     # moves the arm
    .venv/bin/python tools/goto_view.py overview

Holds the code-1 lease, switches suction off first, plans with the workcell envelope and the
configured boxes, keeps the joint-limit margin, and refuses while code-2 is busy.
"""
import argparse
import asyncio
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'tools')]
from connect import connect  # noqa: E402
from viam.components.arm import Arm  # noqa: E402
from viam.components.camera import Camera  # noqa: E402
from viam.components.gripper import Gripper  # noqa: E402
from viam.services.generic import Generic  # noqa: E402
from viam.services.motion import MotionClient  # noqa: E402
from viam.proto.common import GeometriesInFrame, WorldState  # noqa: E402

from pickplace.config import Settings  # noqa: E402
from pickplace.cycle import PickPlaceCycle  # noqa: E402
from pickplace.planning import Planner, box_obstacles, prism  # noqa: E402

VIEWS = {
    'box_view': [0.0, -35.0, -78.57937979708325, 170.0, -95.0, -180.0],  # older box survey view
    'box_top': [-30.0, 0.0, -110.0, 20.0, 120.0, -199.998],  # camera above the box: the accepted box measurement
}


async def main(args):
    s = Settings.from_attributes(json.loads(Path(args.config).read_text()))
    target = s.overview_joints_deg if args.view == 'overview' else VIEWS[args.view]
    async with connect() as robot:
        if (await Generic.from_robot(robot, 'code-2').do_command({'action': 'status'})).get('busy'):
            raise SystemExit('code-2 is busy')
        cycle = PickPlaceCycle(s, Arm.from_robot(robot, 'arm'), Gripper.from_robot(robot, 'Vgripper'), Camera.from_robot(robot, 'cam'),
                               MotionClient.from_robot(robot, 'builtin'), Generic.from_robot(robot, 'code-1'), log=print)
        await cycle.preflight()
        await cycle.acquire_lease()
        try:
            start = await cycle.joints()
            if max(abs(a-b) for a, b in zip(start, target)) > .1:
                await cycle.suction_off(dwell=False)
                world = WorldState(obstacles=[GeometriesInFrame(reference_frame='world', geometries=[
                    prism('scan-objects-envelope', [400, 0, 125], [600, 800, 350])]+box_obstacles(s))])
                path = await Planner(cycle.motion, s, cycle.kin).segment(start, world, goal_deg=target, margin=1.0)
                await cycle.execute('camera_to_'+args.view, path, args.speed)
            await asyncio.sleep(1.0)
            if args.save:
                obs = await cycle.capture(require_ring=False, require_cube=False)
                bgr, depth, props, pose = cycle.last_frame
                out = Path(args.save)
                out.mkdir(parents=True, exist_ok=True)
                import cv2
                import numpy as np
                cv2.imwrite(str(out/'color.png'), bgr)
                np.savez_compressed(out/'depth.npz', depth=depth.astype(np.uint16))
                (out/'meta.json').write_text(json.dumps({'properties': props, 'camera_pose_world': pose,
                                                         'observation_joints_deg': await cycle.joints(), 'issues': obs.get('issues')}, indent=1))
                print('saved', out)
        except BaseException:
            await cycle.stop_arm()
            raise
        finally:
            await cycle.release_lease()


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('view', choices=['overview', *VIEWS])
    p.add_argument('--save')
    p.add_argument('--speed', type=float, default=12.0, help='deg/s for this camera-only move (<= 20)')
    p.add_argument('--config', default=str(ROOT/'config'/'code2-attributes.json'))
    asyncio.run(main(p.parse_args()))
