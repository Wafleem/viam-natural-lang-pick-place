"""Dashboard server against a fake robot: safety gates, allowlists and config helpers."""
import asyncio
import json
import sys
from pathlib import Path

import pytest
from aiohttp.test_utils import TestClient, TestServer

sys.path.insert(0, str(Path(__file__).parent))
import server  # noqa: E402

ATTRS = json.loads((server.MODULE/'tests'/'data'/'attributes-0.0.24.json').read_text())


class FakeBackend:
    def __init__(self, busy=False):
        self.busy, self.calls, self.attributes, self.version = busy, [], json.loads(json.dumps(ATTRS)), '0.0.30'

    async def code2(self, command, timeout=30):
        self.calls.append(('code2', command))
        if command['action'] == 'describe':
            return {'place': {'goals': [{'name': 'ring', 'type': 'ring'}, {'name': 'box', 'type': 'box', 'color': 'orange'}]},
                    'pick': {'select': ['only', 'left', 'right', 'nearest', 'farthest']}, 'default_goal': 'box'}
        return {'state': 'idle', 'busy': self.busy, 'action': command['action']}

    async def arm_state(self):
        return {'joints_deg': [0, 0, -90, 180, -60, 0], 'moving': False, 'tcp_world': {'x': 1, 'y': 2, 'z': 3}}

    async def image(self):
        import cv2
        import numpy as np
        return cv2.imencode('.png', np.full((48, 64, 3), 230, np.uint8))[1].tobytes(), 'image/png'

    async def jog(self, joint, delta, speed):
        self.calls.append(('jog', joint, delta, speed))
        return {'joints_deg': []}

    async def arm_stop(self):
        self.calls.append(('arm_stop',))
        return {'stopped': True}

    async def arm_do(self, command):
        self.calls.append(('arm_do', command))
        return {'ok': True}

    async def gripper(self, action):
        self.calls.append(('gripper', action))
        return {'action': action}

    async def machine_config(self):
        return {'version': self.version, 'attributes': self.attributes}

    async def write_config(self, attributes=None, version=None):
        self.calls.append(('write_config', attributes is not None, version))
        if attributes is not None:
            self.attributes = attributes

    async def logs(self, count, text):
        return [{'time': 't', 'level': 'info', 'logger': 'pickplace', 'message': text}][:count]

    def builds(self, count=6):
        return [{'id': 'b1', 'platform': 'linux/amd64', 'status': 'Done', 'version': '0.0.30', 'time': 'x'},
                {'id': 'b2', 'platform': 'linux/amd64', 'status': 'Building', 'version': '0.0.31', 'time': 'y'}]

    async def deploy(self, version):
        self.calls.append(('deploy', version))
        return {'module_version': version}

    async def plan_check(self, goal, frame, ring_xy, from_current):
        self.calls.append(('plan_check', goal, frame, ring_xy, from_current))
        return {'ok': True, 'stages': []}

    async def close(self):
        pass


def run(backend, scenario):
    async def go():
        async with TestClient(TestServer(server.make_app(backend))) as client:
            return await scenario(client)
    return asyncio.run(go())


def post(client, path, body):
    return client.post(path, json=body)


def test_page_and_state():
    async def scenario(c):
        assert (await c.get('/')).status == 200
        state = await (await c.get('/api/state')).json()
        assert state['joints_deg'][3] == 180 and state['code2']['state'] == 'idle'
        img = await c.get('/api/camera?overlay=1')
        assert img.status == 200 and img.content_type == 'image/jpeg'
    run(FakeBackend(), scenario)


def test_motion_needs_workcell_clear_and_idle():
    fake = FakeBackend()

    async def scenario(c):
        assert (await post(c, '/api/arm/jog', {'joint': 1, 'delta_deg': 2})).status == 400
        assert (await post(c, '/api/gripper', {'action': 'grab'})).status == 400
        assert (await post(c, '/api/arm/jog', {'joint': 1, 'delta_deg': 50, 'workcell_ready': True})).status == 400
        assert (await post(c, '/api/arm/jog', {'joint': 1, 'delta_deg': 2, 'speed_deg_s': 99, 'workcell_ready': True})).status == 200
        assert (await post(c, '/api/gripper', {'action': 'open'})).status == 200  # releasing never needs the box
    run(fake, scenario)
    assert ('jog', 1, 2.0, server.MAX_JOG_SPEED) in fake.calls and ('gripper', 'grab') not in fake.calls

    busy = FakeBackend(busy=True)

    async def busy_scenario(c):
        r = await post(c, '/api/arm/jog', {'joint': 1, 'delta_deg': 2, 'workcell_ready': True})
        assert r.status == 409 and 'busy' in (await r.json())['error']
    run(busy, busy_scenario)
    assert not any(call[0] == 'jog' for call in busy.calls)


def test_arm_diagnostics_allowlist():
    fake = FakeBackend()

    async def scenario(c):
        assert (await post(c, '/api/arm/do', {'command': 'move_gripper'})).status == 400
        assert (await post(c, '/api/arm/do', {'command': 'get_vacuum_state', 'connection_type': 'plugin'})).status == 200
    run(fake, scenario)
    assert ('arm_do', {'get_vacuum_state': True, 'connection_type': 'plugin'}) in fake.calls


def test_config_tuning_and_boxes():
    fake = FakeBackend()

    async def scenario(c):
        r = await post(c, '/api/config/tuning', {'changes': {'press_depth_mm': 25}})
        assert r.status == 400  # outside 0-20
        r = await post(c, '/api/config/tuning', {'changes': {'mystery': 1}})
        assert r.status == 400
        r = await post(c, '/api/config/tuning', {'changes': {'press_depth_mm': 8}, 'apply': True})
        assert r.status == 200 and fake.attributes['press_depth_mm'] == 8
        cfg = await (await c.get('/api/config')).json()
        assert cfg['effective']['press_depth_mm'] == 8 and list(cfg['boxes']) == ['box']
        box = {**cfg['boxes']['box'], 'name': 'box2', 'color': 'blue',
               'floor_center_world_mm': [450.0, 330.0, 0.85], 'placement_center_xy_mm': [450.0, 330.0]}
        assert (await post(c, '/api/config/boxes', {'entry': box, 'apply': True})).status == 200
        assert [b['name'] for b in fake.attributes['boxes']] == ['box', 'box2']
        assert (await post(c, '/api/config/boxes', {'entry': box})).status == 400  # duplicate name
        assert (await post(c, '/api/config/boxes', {'remove': 'box2', 'apply': True})).status == 200
        assert [b['name'] for b in fake.attributes['boxes']] == ['box']
    run(fake, scenario)


def test_versions_deploy_logs_plan_check():
    fake = FakeBackend()

    async def scenario(c):
        v = await (await c.get('/api/versions')).json()
        assert v['current'] == '0.0.30' and len(v['builds']) == 2
        assert (await post(c, '/api/versions/select', {'version': '0.0.31'})).status == 400  # still building
        assert (await post(c, '/api/versions/select', {'version': '0.0.30'})).status == 200
        assert (await post(c, '/api/deploy', {'version': 'latest'})).status == 400
        assert (await post(c, '/api/deploy', {'version': '0.0.31'})).status == 200
        logs = await (await c.get('/api/logs?n=5&filter=error')).json()
        assert logs[0]['message'] == 'error'
        r = await post(c, '/api/plan_check', {'goal': 'ring', 'ring_world_xy': [440, 130], 'start': 'current'})
        assert (await r.json())['ok']
    run(fake, scenario)
    assert ('write_config', False, '0.0.30') in fake.calls and ('deploy', '0.0.31') in fake.calls
    assert ('plan_check', 'ring', 0, [440, 130], True) in fake.calls


def test_nl_uses_live_describe():
    async def scenario(c):
        r = await (await post(c, '/api/nl', {'text': 'put the left cube in the orange box'})).json()
        assert r['command'] == {'action': 'pick_place', 'pick': {'object': 'cube', 'select': 'left'}, 'place': {'color': 'orange'}}
        assert (await post(c, '/api/nl', {'text': 'make coffee'})).status == 400
    run(FakeBackend(), scenario)


def test_add_box_converts_legacy_single_box():
    out = server.add_box(ATTRS, {'name': 'box2', 'floor_center_world_mm': [450, 330, 1], 'inner_dimensions_mm': [260, 210],
                                 'world_yaw_deg': 0, 'rim_z_mm': 130})
    assert [b['name'] for b in out['boxes']] == ['box', 'box2']
    assert out['boxes'][0]['rim_z_mm'] == pytest.approx(ATTRS['surveyed_box']['box']['rim_z_mm'])


def test_box_fit_and_centre():
    fake = FakeBackend()

    async def scenario(c):
        cfg = await (await c.get('/api/config')).json()
        assert cfg['box_fit_mm']['box'] == pytest.approx([30.71, 1.38], abs=0.02)
        assert (await post(c, '/api/config/boxes', {'centre': 'box', 'apply': True})).status == 200
        cfg = await (await c.get('/api/config')).json()
        assert cfg['box_fit_mm']['box'] == pytest.approx([30.71, 3.28], abs=0.02)
        assert 'placement_center_xy_mm' not in fake.attributes['boxes'][0]
    run(fake, scenario)
