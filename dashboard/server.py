"""Local test dashboard for armfarm19: send code-2 commands, jog the arm, switch suction, edit config.

Runs on your laptop at http://localhost:8765 with your own `viam login`; the machine API key
stays in this process's memory and never reaches the browser.

    pickplace-module/.venv/bin/pip install -r dashboard/requirements.txt
    pickplace-module/.venv/bin/python dashboard/server.py

Anything that moves the arm requires the page's "workcell clear" checkbox, and is refused
while code-2 is busy. Jogs are small (<= 10 deg, <= 5 deg/s) direct joint moves without
collision checking: watch the arm.
"""
import argparse
import asyncio
import json
import sys
from pathlib import Path

from aiohttp import web
from google.protobuf.json_format import MessageToDict
from viam.components.arm import Arm
from viam.components.camera import Camera
from viam.components.gripper import Gripper
from viam.proto.component.arm import JointPositions, MoveOptions
from viam.services.generic import Generic
from viam.services.motion import MotionClient

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT/'pickplace-module'
sys.path[:0] = [str(MODULE/'tools'), str(MODULE/'src')]
from connect import app_client, connect  # noqa: E402
from pickplace.config import Settings  # noqa: E402

sys.path.insert(0, str(ROOT/'nl-integration'))
from nl_to_command import DESCRIBE, parse, validate  # noqa: E402

PART_ID = 'f5c74a65-a023-4983-a317-1e9a88c88372'
SERVICE = 'code-2'
MAX_JOG_DEG, MAX_JOG_SPEED = 10.0, 5.0


class Robot:
    """One shared RobotClient, reconnected on failure."""

    def __init__(self):
        self.ctx = self.robot = None
        self.lock = asyncio.Lock()

    async def get(self):
        async with self.lock:
            if self.robot is None:
                self.ctx = connect()
                self.robot = await self.ctx.__aenter__()
            return self.robot

    async def drop(self):
        async with self.lock:
            if self.ctx is not None:
                try:
                    await self.ctx.__aexit__(None, None, None)
                except Exception:
                    pass
            self.ctx = self.robot = None


ROBOT = Robot()


def ok(data):
    return web.json_response(data, dumps=lambda d: json.dumps(d, default=str))


def fail(message, status=400):
    return web.json_response({'error': message}, status=status)


async def call(fn):
    """Run fn(robot); on a connection error reconnect once."""
    for attempt in (0, 1):
        robot = await ROBOT.get()
        try:
            return await fn(robot)
        except (ConnectionError, OSError):
            await ROBOT.drop()
            if attempt:
                raise


async def code2(robot, command, timeout=30):
    return await Generic.from_robot(robot, SERVICE).do_command(command, timeout=timeout)


async def require_idle(robot):
    status = await code2(robot, {'action': 'status'})
    if status.get('busy'):
        raise web.HTTPConflict(text=json.dumps({'error': 'code-2 is busy; stop it first'}), content_type='application/json')


def require_clear(body):
    if body.get('workcell_ready') is not True:
        raise web.HTTPBadRequest(text=json.dumps({'error': 'Tick "workcell clear" first'}), content_type='application/json')


# ---- routes -----------------------------------------------------------------
async def index(_):
    return web.FileResponse(Path(__file__).parent/'static'/'index.html')


async def state(_):
    async def read(robot):
        arm = Arm.from_robot(robot, 'arm')
        out = {'joints_deg': list((await arm.get_joint_positions(timeout=5)).values), 'moving': await arm.is_moving(timeout=5)}
        out['tcp_world'] = MessageToDict(await MotionClient.from_robot(robot, 'builtin').get_pose('Vgripper', 'world', timeout=5))['pose']
        try:
            out['code2'] = await code2(robot, {'action': 'status'}, timeout=10)
        except Exception as e:
            out['code2'] = {'error': str(e)}
        return out
    try:
        return ok(await call(read))
    except Exception as e:
        return fail(f'{type(e).__name__}: {e}', 502)


async def command(request):
    """Pass any JSON command straight to code-2's DoCommand (what the web app will do)."""
    body = await request.json()
    if not isinstance(body, dict) or 'action' not in body:
        return fail('Send a JSON object with an "action"')
    try:
        return ok(await call(lambda r: code2(r, body)))
    except Exception as e:
        return fail(f'{type(e).__name__}: {e}')


async def nl(request):
    """Plain language -> command preview (nothing is sent; the page sends it after review)."""
    body = await request.json()
    try:
        describe = await call(lambda r: code2(r, {'action': 'describe'}))
    except Exception:
        describe = DESCRIBE
    try:
        result = parse(str(body.get('text', '')), describe)
        validate(result['command'], describe)
        return ok(result)
    except Exception as e:
        return fail(str(e))


async def camera(_):
    async def grab(robot):
        images, _ = await Camera.from_robot(robot, 'cam').get_images(timeout=15)
        return next(i for i in images if i.mime_type in ('image/jpeg', 'image/png'))
    try:
        image = await call(grab)
    except Exception as e:
        return fail(f'{type(e).__name__}: {e}', 502)
    return web.Response(body=image.data, content_type=image.mime_type, headers={'Cache-Control': 'no-store'})


async def jog(request):
    body = await request.json()
    require_clear(body)
    joint, delta = int(body.get('joint', -1)), float(body.get('delta_deg', 0))
    speed = min(float(body.get('speed_deg_s', 2)), MAX_JOG_SPEED)
    if not 0 <= joint <= 5 or not 0 < abs(delta) <= MAX_JOG_DEG or speed <= 0:
        return fail(f'joint 0-5, 0 < |delta_deg| <= {MAX_JOG_DEG:g}')

    async def move(robot):
        await require_idle(robot)
        arm = Arm.from_robot(robot, 'arm')
        q = list((await arm.get_joint_positions(timeout=5)).values)
        q[joint] += delta
        await arm.move_through_joint_positions([JointPositions(values=q)], options=MoveOptions(
            max_vel_degs_per_sec=speed, max_acc_degs_per_sec2=6), timeout=60)
        return {'joints_deg': list((await arm.get_joint_positions(timeout=5)).values)}
    try:
        return ok(await call(move))
    except web.HTTPException:
        raise
    except Exception as e:
        return fail(f'{type(e).__name__}: {e}')


ARM_DIAGNOSTICS = {'get_state', 'get_error', 'clear_error', 'get_vacuum_state'}


async def arm_do(request):
    """viam:ufactory driver DoCommands that never move the arm: state, error, clear_error and
    vacuum state (optionally with connection_type plugin/contact to compare the wiring)."""
    body = await request.json()
    key = body.get('command')
    if key not in ARM_DIAGNOSTICS:
        return fail('command must be one of '+', '.join(sorted(ARM_DIAGNOSTICS)))
    cmd = {key: True}
    if key == 'get_vacuum_state' and body.get('connection_type') in ('plugin', 'contact'):
        cmd['connection_type'] = body['connection_type']
    try:
        return ok(await call(lambda r: Arm.from_robot(r, 'arm').do_command(cmd, timeout=10)))
    except Exception as e:
        return fail(f'{type(e).__name__}: {e}')


async def arm_stop(_):
    try:
        await call(lambda r: Arm.from_robot(r, 'arm').stop(timeout=5))
        return ok({'stopped': True})
    except Exception as e:
        return fail(f'{type(e).__name__}: {e}')


async def gripper(request):
    body = await request.json()
    action = body.get('action')
    if action not in ('grab', 'open'):
        return fail('action must be grab or open')

    async def act(robot):
        await require_idle(robot)
        g = Gripper.from_robot(robot, 'Vgripper')
        result = await (g.grab(timeout=5) if action == 'grab' else g.open(timeout=5))
        return {'action': action, 'result': result}
    try:
        return ok(await call(act))
    except web.HTTPException:
        raise
    except Exception as e:
        return fail(f'{type(e).__name__}: {e}')


async def get_config(_):
    async with app_client() as app:
        part = await app.get_robot_part(PART_ID)
    cfg = part.robot_config
    service = next(s for s in cfg['services'] if s.get('name') == SERVICE)
    module = next(m for m in cfg['modules'] if m.get('module_id') == 'hackathons:armfarm19-vision-pickplace')
    return ok({'version': module.get('version'), 'attributes': service.get('attributes', {})})


async def put_config(request):
    """Validate (dry) or apply new code-2 attributes. Only code-2's attributes change."""
    body = await request.json()
    attributes = body.get('attributes')
    try:
        settings = Settings.from_attributes(attributes)
    except Exception as e:
        return fail(f'Invalid attributes: {e}')
    summary = {'valid': True, 'goals': settings.goals, 'press_depth_mm': settings.press_depth_mm}
    if not body.get('apply'):
        return ok(summary)
    try:
        await call(require_idle)
    except web.HTTPException:
        raise
    async with app_client() as app:
        part = await app.get_robot_part(PART_ID)
        cfg = part.robot_config
        next(s for s in cfg['services'] if s.get('name') == SERVICE)['attributes'] = attributes
        await app.update_robot_part(part.id, part.name, cfg, last_known_update=part.last_updated)
    return ok({**summary, 'applied': True, 'note': 'code-2 restarts with the new attributes in a few seconds'})


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', type=int, default=8765)
    args = p.parse_args()
    app = web.Application()
    app.add_routes([web.get('/', index), web.get('/api/state', state), web.post('/api/command', command), web.post('/api/nl', nl),
                    web.get('/api/camera', camera), web.post('/api/arm/jog', jog), web.post('/api/arm/stop', arm_stop), web.post('/api/arm/do', arm_do),
                    web.post('/api/gripper', gripper), web.get('/api/config', get_config), web.post('/api/config', put_config)])
    app.on_cleanup.append(lambda _: ROBOT.drop())
    print(f'Dashboard: http://localhost:{args.port}')
    web.run_app(app, host='127.0.0.1', port=args.port, print=None)


if __name__ == '__main__':
    main()
