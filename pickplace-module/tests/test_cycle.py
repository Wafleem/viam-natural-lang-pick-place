"""Full cycle against fake Viam resources: planning + execution order, no real motion."""
import asyncio
import json
import math
import time

import cv2
import numpy as np
import pytest
from google.protobuf.json_format import ParseDict
from google.protobuf.timestamp_pb2 import Timestamp
from viam.proto.common import Pose, PoseInFrame, ResponseMetadata
from viam.proto.component.camera import GetPropertiesResponse

from conftest import FakeMotion, load_frame
from pickplace.cycle import CycleError, PickPlaceCycle
from pickplace.planning import PlanError
from pickplace.transforms import matrix_pose

FAILED_LIFT = [32.46712663473719, -1.157959002424168, -53.693261122156215, 360.0007749990997, 55.16356179383901, -329.16006666990427]


def pif(frame, d):
    keys = {'x': 'x', 'y': 'y', 'z': 'z', 'o_x': 'o_x', 'o_y': 'o_y', 'o_z': 'o_z', 'oX': 'o_x', 'oY': 'o_y', 'oZ': 'o_z', 'theta': 'theta'}
    return PoseInFrame(reference_frame=frame, pose=Pose(**{keys[k]: float(v) for k, v in d.items() if k in keys}))


class FakeArm:
    def __init__(self, urdf, joints):
        self.urdf, self.q, self.moves = urdf, list(joints), []

    async def get_joint_positions(self, timeout=None):
        return type('J', (), {'values': list(self.q)})()

    async def get_kinematics(self, timeout=None):
        return (2, self.urdf.encode(), {})

    async def is_moving(self, timeout=None):
        return False

    async def move_through_joint_positions(self, positions, options=None, timeout=None):
        path = [list(p.values) for p in positions]
        self.moves.append((path, options.max_vel_degs_per_sec))
        self.q = path[-1]

    async def stop(self, timeout=None):
        self.moves.append(('stop', None))


class FakeGripper:
    def __init__(self):
        self.calls = []

    async def grab(self, timeout=None):
        self.calls.append('grab')
        return True

    async def open(self, timeout=None):
        self.calls.append('open')

    async def is_holding_something(self, *a, **k):
        raise AssertionError('Holding feedback must never be read (open-loop suction)')


class FakeCamera:
    def __init__(self):
        meta, bgr, depth = load_frame(0)
        self.meta = meta
        self.png = cv2.imencode('.png', bgr)[1].tobytes()
        d = depth.astype('>u2')
        self.dep = b'DEPTHMAP'+d.shape[1].to_bytes(8, 'big')+d.shape[0].to_bytes(8, 'big')+d.tobytes()
        self.count = 0

    async def get_properties(self, timeout=None):
        return ParseDict(self.meta['properties'], GetPropertiesResponse(), ignore_unknown_fields=True)

    async def get_images(self, timeout=None):
        self.count += 1
        stamp = Timestamp()
        stamp.FromNanoseconds(time.time_ns()+self.count*1000)
        image = lambda mime, data: type('I', (), {'mime_type': mime, 'data': data})()
        return [image('image/png', self.png), image('image/vnd.viam.dep', self.dep)], ResponseMetadata(captured_at=stamp)


class FakeLease:
    def __init__(self):
        self.reserved = ''

    async def do_command(self, command, timeout=None):
        if command['action'] == 'acquire_lease':
            self.reserved = command['token']
            return {'reserved': True}
        if command['action'] == 'release_lease':
            self.reserved = ''
            return {}
        return {'busy': False, 'reserved_by': self.reserved}


def make_cycle(settings, kin, urdf, joints):
    arm, gripper, camera, lease = FakeArm(urdf, joints), FakeGripper(), FakeCamera(), FakeLease()
    motion = FakeMotion(kin)
    motion.poses = {
        ('Vgripper', 'arm'): lambda: pif('arm', {'z': 126, 'o_z': 1}),
        ('Vgripper', 'world'): lambda: pif('world', matrix_pose(kin.fk(arm.q).tolist())),
        ('cam', 'arm'): lambda: pif('arm', settings.camera_mount_reference),
        ('cam', 'world'): lambda: pif('world', camera.meta['camera_pose_world']),
    }
    return PickPlaceCycle(settings, arm, gripper, camera, motion, lease), arm, gripper, motion, lease


@pytest.mark.parametrize('goal', ['box', 'ring'])
def test_full_cycle_open_loop_from_overview(settings, kin, urdf, goal):
    cycle, arm, gripper, motion, lease = make_cycle(settings, kin, urdf, settings.overview_joints_deg)
    summary = asyncio.run(cycle.run(goal))
    stages = [e['stage'] for e in cycle.events]
    assert stages[0] == 'suction_off'  # every run starts by releasing suction
    stages = stages[stages.index('detecting'):]
    order = ['approach', 'descend', 'suction_on', 'press', 'suction_dwell', 'lift', 'transfer', 'place', 'suction_off', 'retract', 'return', 'complete']
    assert [s for s in stages if s in order] == order
    assert gripper.calls == ['open', 'grab', 'open']  # reset release, grab before press, release at goal
    speeds = dict(zip([s for s in stages if s in ('approach', 'descend', 'press', 'lift', 'transfer', 'place', 'retract', 'return')], [m[1] for m in arm.moves]))
    assert speeds['press'] == settings.press_speed_deg_s and speeds['transfer'] == settings.travel_speed_deg_s
    # The press stage ends exactly press_depth_mm below the taught contact.
    press_path = arm.moves[2][0]
    assert kin.fk(press_path[-1])[2, 3] == pytest.approx(settings.calibration['pickup_tcp_z_mm']-10, abs=.05)
    # No executed node comes near a limit except the calibrated overview itself (J5 at -95).
    for path, _ in arm.moves:
        for q in path:
            assert kin.within_limits(q, .5)
            assert abs(q[3]) < 360-settings.joint_limit_margin_deg
    assert arm.q == pytest.approx(settings.overview_joints_deg)
    assert summary['press_depth_mm'] == 10 and lease.reserved == ''
    json.dumps(summary)
    assert all(type(v) is float for v in summary['pickup_tcp_world'].values())
    press_request = motion.requests[2]
    allowed = {(a['frame1'], a['frame2']) for a in press_request['constraints']['collision_specification'][0]['allows']}
    assert ('Vgripper', 'vision-cube') in allowed and ('vision-cube', 'vision-table') in allowed


def test_cycle_from_low_pose_retreats_first(settings, kin, urdf):
    # A stop mid-pickup, on the cycle's own wrist branch (J4 near 0).
    low = kin.ik({'x': 380, 'y': 240, 'z': 250, 'o_z': -1, 'theta': 0}, [32, -3, -90, 0, 93, -229])
    cycle, arm, gripper, motion, _ = make_cycle(settings, kin, urdf, low)
    asyncio.run(cycle.reset())
    assert [e['stage'] for e in cycle.events if e['stage'].startswith('reset')] == ['reset_retreat', 'reset_to_overview']
    retreat = arm.moves[0][0]
    assert kin.fk(retreat[-1])[2, 3] == pytest.approx(360, abs=.1)
    assert np.allclose(kin.fk(retreat[-1])[:2, 3], kin.fk(low)[:2, 3], atol=.1)
    assert gripper.calls == ['open']


def test_joint_at_limit_requires_recovery(settings, kin, urdf):
    cycle, arm, gripper, _, _ = make_cycle(settings, kin, urdf, FAILED_LIFT)
    with pytest.raises(PlanError, match='recover_limits'):
        asyncio.run(cycle.run('box'))
    assert arm.moves[-1] == ('stop', None)
    result = asyncio.run(cycle.recover_limits())
    assert result['to'][3] == pytest.approx(360-settings.joint_limit_margin_deg)
    assert result['to'][:3] == pytest.approx(FAILED_LIFT[:3]) and result['to'][4:] == pytest.approx(FAILED_LIFT[4:])
    assert arm.moves[-1][1] == 2


def test_planner_rejects_paths_at_the_limit(settings, kin):
    from pickplace.planning import Planner
    planner = Planner(None, settings, kin)
    start = kin.ik({'x': 378.9, 'y': 241.1, 'z': 430, 'o_z': -1, 'theta': -1.6}, settings.overview_joints_deg)
    at_limit = list(start)
    at_limit[3] = 360.0
    reply = {'plan': [{'arm': [math.radians(v) for v in q]} for q in (start, at_limit)]}
    with pytest.raises(PlanError, match='joint limit'):
        planner.validate(reply, start)
    ok = {'plan': [{'arm': [math.radians(v) for v in q]} for q in (settings.overview_joints_deg, start)]}
    assert planner.validate(ok, settings.overview_joints_deg, start)[-1] == pytest.approx(start)


def test_service_command_guards(settings, kin, urdf, attrs):
    from pickplace.service import PickPlaceService
    svc = PickPlaceService('code-2')
    svc.task, svc.state, svc.error, svc.last_result, svc.request_id, svc.used_requests = None, 'idle', '', {}, '', set()
    svc.settings = settings
    svc.cycle, *_ = make_cycle(settings, kin, urdf, settings.overview_joints_deg)
    with pytest.raises(ValueError, match='workcell_ready'):
        asyncio.run(svc.do_command({'action': 'run', 'goal': 'box'}))
    with pytest.raises(ValueError, match='Unknown action'):
        asyncio.run(svc.do_command({'action': 'start'}))
    status = asyncio.run(svc.do_command({'action': 'status'}))
    assert status['suction'] == {'mode': 'open_loop', 'holding_feedback_used': False, 'press_depth_mm': 10.0,
                                 'suction_dwell_s': 1.0, 'release_dwell_s': 1.0}
    json.dumps(status)


def test_no_holding_feedback_in_source():
    from pathlib import Path
    src = Path(__file__).resolve().parents[1]/'src'
    for path in src.rglob('*.py'):
        assert 'is_holding_something' not in path.read_text(), path
