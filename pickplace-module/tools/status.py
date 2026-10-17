"""Read-only snapshot: code-2 status, arm joints/TCP, joint-limit clearance. Never moves anything.

    .venv/bin/python tools/status.py
"""
import asyncio
import json
import sys
from pathlib import Path

from google.protobuf.json_format import MessageToDict
from viam.components.arm import Arm
from viam.services.generic import Generic
from viam.services.motion import MotionClient

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'tools')]
from connect import connect  # noqa: E402
from pickplace.kinematics import ArmKinematics  # noqa: E402


async def main():
    async with connect() as robot:
        arm = Arm.from_robot(robot, 'arm')
        q = list((await arm.get_joint_positions(timeout=5)).values)
        kind, raw = (await arm.get_kinematics(timeout=10))[:2]
        report = {'joints_deg': [round(v, 3) for v in q], 'moving': await arm.is_moving(timeout=5)}
        if kind == 2:
            kin = ArmKinematics(raw.decode())
            report['min_joint_limit_clearance_deg'] = round(kin.limit_clearance_deg(q), 3)
        report['tcp_world'] = MessageToDict(await MotionClient.from_robot(robot, 'builtin').get_pose('Vgripper', 'world', timeout=5))['pose']
        status = await Generic.from_robot(robot, 'code-2').do_command({'action': 'status'}, timeout=10)
        keep = ('implementation', 'state', 'stage', 'busy', 'error', 'suction', 'last_result')
        report['code-2'] = {k: status.get(k) for k in keep if k in status}
        print(json.dumps(report, indent=1, default=str))


if __name__ == '__main__':
    asyncio.run(main())
