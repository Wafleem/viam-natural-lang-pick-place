"""Ultraviolet test bench for armfarm19: every code-2 command, arm/gripper/controller debug,
vision overlay, live plan checks, config editing, logs and module versions on one page.

Runs on your laptop at http://localhost:8765 with your own `viam login`; the machine API key
stays in this process and never reaches the browser.

    make dashboard            (or: pickplace-module/.venv/bin/python dashboard/server.py)

Anything that moves the arm or switches suction on needs the page's "workcell clear" box, and is
refused while code-2 is busy. Jogs are small direct joint moves without collision checking.
"""
import argparse
import asyncio
import json
import os
import subprocess
import sys
from pathlib import Path

from aiohttp import web

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT/'pickplace-module'
sys.path[:0] = [str(MODULE/'tools'), str(MODULE/'src'), str(MODULE/'tests'), str(ROOT/'nl-integration')]
from pickplace.config import Settings  # noqa: E402
from nl_to_command import DESCRIBE, parse, validate  # noqa: E402

PART_ID = 'f5c74a65-a023-4983-a317-1e9a88c88372'
SERVICE, MODULE_ID = 'code-2', 'hackathons:armfarm19-vision-pickplace'
MAX_JOG_DEG, MAX_JOG_SPEED = 10.0, 5.0
ARM_DIAGNOSTICS = {'get_state', 'get_error', 'clear_error', 'get_vacuum_state'}
# Config keys the tuning form may set; the module validates their bounds.
TUNING_KEYS = ('press_depth_mm', 'pre_contact_mm', 'suction_dwell_s', 'release_dwell_s', 'clearance_mm',
               'box_entry_clearance_mm', 'release_above_goal_mm', 'travel_speed_deg_s', 'approach_speed_deg_s',
               'press_speed_deg_s', 'acceleration_deg_s2', 'joint_limit_margin_deg', 'default_goal')
BOX_KEYS = ('floor_center_world_mm', 'placement_center_xy_mm', 'inner_dimensions_mm', 'world_yaw_deg', 'rim_z_mm')


class Refused(Exception):
    """A request the bench will not perform (bad input, unsafe while busy, not confirmed)."""

    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


# ---- config helpers (pure; unit-tested) --------------------------------------------------
def apply_tuning(attributes, changes):
    """Copy of the attributes with whitelisted tuning keys changed ('' removes = default)."""
    unknown = sorted(set(changes)-set(TUNING_KEYS))
    if unknown:
        raise Refused('Not a tuning key: '+', '.join(unknown))
    result = json.loads(json.dumps(attributes))
    for key, value in changes.items():
        if value in (None, ''):
            result.pop(key, None)
        else:
            result[key] = value if key == 'default_goal' else float(value)
    Settings.from_attributes(result)
    return result


def box_list(attributes):
    """attributes['boxes'] (in place), creating it from the legacy single box when needed."""
    if attributes.get('boxes') is None:
        legacy = attributes.pop('box', None) or (attributes.get('surveyed_box') or {}).get('box')
        attributes['boxes'] = [{'name': 'box', **{k: legacy[k] for k in BOX_KEYS if k in legacy}}] if legacy else []
    return attributes['boxes']


def add_box(attributes, entry):
    """Copy of the attributes with one more box; turns the legacy single box into a list first."""
    result = json.loads(json.dumps(attributes))
    boxes = box_list(result)
    if any(b.get('name') == entry.get('name') for b in boxes):
        raise Refused(f'A box named {entry.get("name")!r} already exists')
    boxes.append(entry)
    Settings.from_attributes(result)
    return result


def centre_box(attributes, name):
    """Drop a box's placement_center_xy_mm so the cube goes to the measured floor centre."""
    result = json.loads(json.dumps(attributes))
    match = [b for b in box_list(result) if b.get('name') == name]
    if not match:
        raise Refused(f'No box named {name!r}')
    match[0].pop('placement_center_xy_mm', None)
    Settings.from_attributes(result)
    return result


def box_fit(settings):
    """Per-box gap (mm per side, long and short axis) for the cube placed square at its goal."""
    from pickplace.box import check_cube_fits, placement_xy
    out = {}
    for name, b in settings.boxes.items():
        try:
            out[name] = [round(g, 2) for g in check_cube_fits(b, settings.cube_size_mm, placement_xy(b), b['world_yaw_deg'])]
        except ValueError as e:
            out[name] = str(e)
    return out


def remove_box(attributes, name):
    result = json.loads(json.dumps(attributes))
    if not isinstance(result.get('boxes'), list) or not any(b.get('name') == name for b in result['boxes']):
        raise Refused(f'No box named {name!r} in the boxes list')
    result['boxes'] = [b for b in result['boxes'] if b.get('name') != name]
    if result.get('default_goal') == name:
        result['default_goal'] = 'ring'
    Settings.from_attributes(result)
    return result


# ---- the robot backend --------------------------------------------------------------------
class RobotBackend:
    """Everything the bench does to the robot or the Viam app, in one place (a fake in tests)."""

    def __init__(self, cli):
        self.ctx = self.robot = None
        self.lock = asyncio.Lock()
        self.cli = cli

    async def _robot(self):
        from connect import connect
        async with self.lock:
            if self.robot is None:
                self.ctx = connect()
                self.robot = await self.ctx.__aenter__()
            return self.robot

    async def close(self):
        async with self.lock:
            if self.ctx is not None:
                try:
                    await self.ctx.__aexit__(None, None, None)
                except Exception:
                    pass
            self.ctx = self.robot = None

    async def _with(self, fn):
        for attempt in (0, 1):
            robot = await self._robot()
            try:
                return await fn(robot)
            except Exception as e:
                # Lost connection, or a resource list taken while a module was restarting.
                if not isinstance(e, (ConnectionError, OSError)) and type(e).__name__ != 'ResourceNotFoundError':
                    raise
                await self.close()
                if attempt:
                    raise

    async def code2(self, command, timeout=30):
        from viam.services.generic import Generic
        return await self._with(lambda r: Generic.from_robot(r, SERVICE).do_command(command, timeout=timeout))

    async def arm_state(self):
        from google.protobuf.json_format import MessageToDict
        from viam.components.arm import Arm
        from viam.services.motion import MotionClient

        async def read(robot):
            arm = Arm.from_robot(robot, 'arm')
            return {'joints_deg': list((await arm.get_joint_positions(timeout=5)).values),
                    'moving': await arm.is_moving(timeout=5),
                    'tcp_world': MessageToDict(await MotionClient.from_robot(robot, 'builtin').get_pose('Vgripper', 'world', timeout=5))['pose']}
        return await self._with(read)

    async def image(self):
        from viam.components.camera import Camera

        async def grab(robot):
            images, _ = await Camera.from_robot(robot, 'cam').get_images(timeout=15)
            return next(i for i in images if i.mime_type in ('image/jpeg', 'image/png'))
        image = await self._with(grab)
        return image.data, image.mime_type

    async def jog(self, joint, delta, speed):
        from viam.components.arm import Arm
        from viam.proto.component.arm import JointPositions, MoveOptions

        async def move(robot):
            arm = Arm.from_robot(robot, 'arm')
            q = list((await arm.get_joint_positions(timeout=5)).values)
            q[joint] += delta
            await arm.move_through_joint_positions([JointPositions(values=q)], options=MoveOptions(
                max_vel_degs_per_sec=speed, max_acc_degs_per_sec2=6), timeout=60)
            return {'joints_deg': list((await arm.get_joint_positions(timeout=5)).values)}
        return await self._with(move)

    async def arm_stop(self):
        from viam.components.arm import Arm
        await self._with(lambda r: Arm.from_robot(r, 'arm').stop(timeout=5))
        return {'stopped': True}

    async def arm_do(self, command):
        from viam.components.arm import Arm
        return await self._with(lambda r: Arm.from_robot(r, 'arm').do_command(command, timeout=10))

    async def gripper(self, action):
        from viam.components.gripper import Gripper

        async def act(robot):
            g = Gripper.from_robot(robot, 'Vgripper')
            return {'action': action, 'result': await (g.grab(timeout=5) if action == 'grab' else g.open(timeout=5))}
        return await self._with(act)

    async def machine_config(self):
        from connect import app_client
        async with app_client() as app:
            part = await app.get_robot_part(PART_ID)
        cfg = part.robot_config
        service = next(s for s in cfg['services'] if s.get('name') == SERVICE)
        module = next(m for m in cfg['modules'] if m.get('module_id') == MODULE_ID)
        return {'version': module.get('version'), 'attributes': service.get('attributes', {})}

    async def write_config(self, attributes=None, version=None):
        """Replace code-2's attributes and/or its module version; nothing else in the config."""
        from connect import app_client
        async with app_client() as app:
            part = await app.get_robot_part(PART_ID)
            cfg = part.robot_config
            if attributes is not None:
                next(s for s in cfg['services'] if s.get('name') == SERVICE)['attributes'] = attributes
            if version is not None:
                next(m for m in cfg['modules'] if m.get('module_id') == MODULE_ID)['version'] = version
            await app.update_robot_part(part.id, part.name, cfg, last_known_update=part.last_updated)

    async def logs(self, count, text):
        from connect import app_client
        async with app_client() as app:
            entries = await app.get_robot_part_logs(PART_ID, filter=text or None, num_log_entries=count)
        return [{'time': e.time.isoformat(timespec='seconds') if e.time else '',
                 'level': e.level, 'logger': e.logger_name, 'message': e.message} for e in entries]

    def builds(self, count=6):
        out = subprocess.run([self.cli, '--config', str(Path.home()/'.viam'/'cached_cli_config.json'), 'module', 'build', 'list',
                              '--module', str(MODULE/'meta.json'), '--count', str(count)], capture_output=True, text=True, timeout=60)
        rows = []
        for line in out.stdout.splitlines()[1:]:
            parts = line.split()
            if len(parts) >= 5:
                rows.append({'id': parts[0], 'platform': parts[1], 'status': parts[2], 'version': parts[3], 'time': parts[4]})
        return sorted(rows, key=lambda r: r['time'], reverse=True)  # newest first

    async def deploy(self, version):
        import deploy
        await deploy.upload(argparse.Namespace(version=version))
        return json.loads((MODULE/'tools'/'last-build.json').read_text())

    async def plan_check(self, goal, frame, ring_xy, from_current):
        """Live planner, plan only (as tools/plan_check.py). Uses a saved overview frame."""
        import plan_check
        from viam.components.arm import Arm
        from viam.components.camera import Camera
        from viam.services.motion import MotionClient
        from pickplace.cycle import PickPlaceCycle
        attrs = (await self.machine_config())['attributes']
        s = Settings.from_attributes(attrs)

        async def run(robot):
            cycle = PickPlaceCycle(s, plan_check.ReadOnlyArm(Arm.from_robot(robot, 'arm')), None, Camera.from_robot(robot, 'cam'),
                                   plan_check.PlanOnly(MotionClient.from_robot(robot, 'builtin')), None)
            kin = await cycle.load_kinematics()
            current = await cycle.joints()
            obs = plan_check.saved_observation(attrs, frame, goal == 'ring')
            if ring_xy:
                plan_check.move_ring(obs, s.calibration, ring_xy)
            plans = await cycle.plan(goal, obs, current if from_current else s.overview_joints_deg)
            return {'ok': True, 'summary': cycle.plan_summary,
                    'stages': [{'name': n, 'nodes': len(p), 'speed_deg_s': v,
                                'j4_range_deg': [round(min(q[3] for q in p), 1), round(max(q[3] for q in p), 1)]} for n, p, v in plans],
                    'current_min_limit_clearance_deg': round(kin.limit_clearance_deg(current), 3)}
        try:
            return await self._with(run)
        except Exception as e:
            return {'ok': False, 'error': f'{type(e).__name__}: {e}'}


# ---- routes ---------------------------------------------------------------------------------
def ok(data):
    return web.json_response(data, dumps=lambda d: json.dumps(d, default=str))


def guarded(handler):
    async def wrapped(request):
        try:
            return await handler(request)
        except Refused as e:
            return web.json_response({'error': str(e)}, status=e.status)
        except web.HTTPException:
            raise
        except Exception as e:
            return web.json_response({'error': f'{type(e).__name__}: {e}'}, status=502)
    return wrapped


async def body_of(request):
    try:
        body = await request.json()
    except Exception:
        raise Refused('Send a JSON body')
    if not isinstance(body, dict):
        raise Refused('Send a JSON object')
    return body


def make_app(backend):
    routes = web.RouteTableDef()

    def need_clear(body):
        if body.get('workcell_ready') is not True:
            raise Refused('Tick "workcell clear" first')

    async def need_idle():
        if (await backend.code2({'action': 'status'})).get('busy'):
            raise Refused('code-2 is busy; stop it first', 409)

    @routes.get('/')
    async def index(_):
        return web.FileResponse(Path(__file__).parent/'static'/'index.html')

    @routes.get('/api/state')
    @guarded
    async def state(_):
        out = await backend.arm_state()
        try:
            out['code2'] = await backend.code2({'action': 'status'}, timeout=10)
        except Exception as e:
            out['code2'] = {'error': str(e)}
        return ok(out)

    @routes.post('/api/command')
    @guarded
    async def command(request):
        body = await body_of(request)
        if 'action' not in body:
            raise Refused('Send a JSON object with an "action"')
        return ok(await backend.code2(body))

    @routes.post('/api/nl')
    @guarded
    async def nl(request):
        body = await body_of(request)
        try:
            describe = await backend.code2({'action': 'describe'})
        except Exception:
            describe = DESCRIBE
        try:
            result = parse(str(body.get('text', '')), describe)
            validate(result['command'], describe)
        except ValueError as e:
            raise Refused(str(e))
        return ok(result)

    @routes.get('/api/camera')
    @guarded
    async def camera(request):
        data, mime = await backend.image()
        if request.query.get('overlay') == '1':
            import cv2
            import numpy as np
            from pickplace.find_cube_ring import annotate, find_objects
            bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            cubes, rings = find_objects(bgr)
            out = annotate(bgr, cubes, rings)
            cv2.putText(out, f'{len(cubes)} cube(s), {len(rings)} ring(s)', (20, 44), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 90, 190), 3)
            data, mime = cv2.imencode('.jpg', out)[1].tobytes(), 'image/jpeg'
        return web.Response(body=data, content_type=mime, headers={'Cache-Control': 'no-store'})

    @routes.post('/api/arm/jog')
    @guarded
    async def jog(request):
        body = await body_of(request)
        need_clear(body)
        joint, delta = int(body.get('joint', -1)), float(body.get('delta_deg', 0))
        speed = min(float(body.get('speed_deg_s', 2)), MAX_JOG_SPEED)
        if not 0 <= joint <= 5 or not 0 < abs(delta) <= MAX_JOG_DEG or speed <= 0:
            raise Refused(f'joint 0-5, 0 < |delta_deg| <= {MAX_JOG_DEG:g}')
        await need_idle()
        return ok(await backend.jog(joint, delta, speed))

    @routes.post('/api/arm/stop')
    @guarded
    async def arm_stop(_):
        return ok(await backend.arm_stop())

    @routes.post('/api/arm/do')
    @guarded
    async def arm_do(request):
        body = await body_of(request)
        key = body.get('command')
        if key not in ARM_DIAGNOSTICS:
            raise Refused('command must be one of '+', '.join(sorted(ARM_DIAGNOSTICS)))
        cmd = {key: True}
        if key == 'get_vacuum_state' and body.get('connection_type') in ('plugin', 'contact'):
            cmd['connection_type'] = body['connection_type']
        return ok(await backend.arm_do(cmd))

    @routes.post('/api/gripper')
    @guarded
    async def gripper(request):
        body = await body_of(request)
        action = body.get('action')
        if action not in ('grab', 'open'):
            raise Refused('action must be grab or open')
        if action == 'grab':
            need_clear(body)
        await need_idle()
        return ok(await backend.gripper(action))

    @routes.get('/api/config')
    @guarded
    async def get_config(_):
        cfg = await backend.machine_config()
        s = Settings.from_attributes(cfg['attributes'])
        cfg['effective'] = {k: getattr(s, k) for k in TUNING_KEYS}
        cfg['boxes'] = s.boxes
        cfg['box_fit_mm'] = box_fit(s)
        return ok(cfg)

    async def save(attributes, apply):
        s = Settings.from_attributes(attributes)
        summary = {'valid': True, 'goals': s.goals, 'press_depth_mm': s.press_depth_mm}
        if apply:
            await need_idle()
            await backend.write_config(attributes=attributes)
            summary.update(applied=True, note='code-2 restarts with the new attributes in a few seconds')
        return ok(summary)

    @routes.post('/api/config')
    @guarded
    async def put_config(request):
        body = await body_of(request)
        try:
            return await save(body.get('attributes'), body.get('apply') is True)
        except (ValueError, TypeError, AttributeError) as e:
            raise Refused(f'Invalid attributes: {e}')

    @routes.post('/api/config/tuning')
    @guarded
    async def tuning(request):
        body = await body_of(request)
        current = (await backend.machine_config())['attributes']
        try:
            return await save(apply_tuning(current, body.get('changes', {})), body.get('apply') is True)
        except (ValueError, TypeError) as e:
            raise Refused(f'Invalid tuning: {e}')

    @routes.post('/api/config/boxes')
    @guarded
    async def boxes(request):
        body = await body_of(request)
        current = (await backend.machine_config())['attributes']
        try:
            if body.get('remove'):
                changed = remove_box(current, str(body['remove']))
            elif body.get('centre'):
                changed = centre_box(current, str(body['centre']))
            else:
                changed = add_box(current, body.get('entry', {}))
            return await save(changed, body.get('apply') is True)
        except (ValueError, TypeError, KeyError) as e:
            raise Refused(f'Invalid box: {e}')

    @routes.get('/api/logs')
    @guarded
    async def logs(request):
        count = max(1, min(int(request.query.get('n', 60)), 500))
        return ok(await backend.logs(count, request.query.get('filter', '')))

    @routes.get('/api/versions')
    @guarded
    async def versions(_):
        current = (await backend.machine_config())['version']
        return ok({'current': current, 'builds': await asyncio.to_thread(backend.builds)})

    @routes.post('/api/versions/select')
    @guarded
    async def select_version(request):
        body = await body_of(request)
        version = str(body.get('version', ''))
        builds = await asyncio.to_thread(backend.builds, 30)
        if not any(b['version'] == version and b['status'] == 'Done' for b in builds):
            raise Refused(f'No finished build for version {version!r}')
        await need_idle()
        await backend.write_config(version=version)
        return ok({'selected': version, 'note': 'code-2 reloads in a few seconds'})

    @routes.post('/api/deploy')
    @guarded
    async def deploy(request):
        body = await body_of(request)
        version = str(body.get('version', ''))
        if version.count('.') != 2 or not all(p.isdigit() for p in version.split('.')):
            raise Refused('version must look like 0.0.31')
        return ok(await backend.deploy(version))

    @routes.post('/api/plan_check')
    @guarded
    async def plan_check(request):
        body = await body_of(request)
        ring = body.get('ring_world_xy')
        if ring is not None and (not isinstance(ring, list) or len(ring) != 2):
            raise Refused('ring_world_xy must be [x, y]')
        return ok(await backend.plan_check(str(body.get('goal', 'box')), int(body.get('frame', 0)), ring,
                                           body.get('start') == 'current'))

    app = web.Application()
    app.add_routes(routes)
    app.on_cleanup.append(lambda _: backend.close())
    return app


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', type=int, default=8765)
    args = p.parse_args()
    print(f'Ultraviolet bench: http://localhost:{args.port}')
    web.run_app(make_app(RobotBackend(os.environ.get('VIAM_CLI', '/private/tmp/viam-armfarm19'))),
                host='127.0.0.1', port=args.port, print=None)


if __name__ == '__main__':
    main()
