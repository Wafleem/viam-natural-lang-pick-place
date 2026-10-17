"""Viam generic service `code-2`: DoCommand front end for PickPlaceCycle.

Commands (JSON in the Viam app CONTROL tab -> code-2 -> DoCommand, or any Viam SDK's
GenericService.do_command; {"action":"describe"} returns this list machine-readably):
  {"action": "describe"}                                       API description for UIs
  {"action": "status"}                                         read only
  {"action": "pick_place", "pick": {"object": "cube", "select": "left"},
   "place": {"goal": "box"} | {"type": "ring"} | {"color": "orange"}, "workcell_ready": true}   (MOVES)
  {"action": "dry_run", "goal": "box"}                         detect + plan, no motion (at overview)
  {"action": "run", "goal": "ring"|<box name>, "workcell_ready": true}   one full cycle (MOVES)
  {"action": "box_once", "workcell_ready": true}               alias: run goal=<first configured box>
  {"action": "ring_once", "workcell_ready": true}              alias: run goal=ring
  {"action": "reset", "workcell_ready": true}                  suction off, go to overview (MOVES)
  {"action": "recover_limits", "workcell_ready": true}         back off a joint stuck at its limit (MOVES)
  {"action": "measure_box", "name": "box2"}                    no motion (at overview): candidate box entry
  {"action": "self_test"}                                      no motion: kinematics, camera, lease, reset plan
  {"action": "release"}                                        suction off, no arm motion
  {"action": "stop"}                                           cancel the running command, stop the arm
Long commands run in the background; poll status. A new command is refused while busy.
"""
import asyncio
import copy
import uuid
from typing import ClassVar, Mapping, Optional, Sequence, Tuple

from viam.components.arm import Arm
from viam.components.camera import Camera
from viam.components.gripper import Gripper
from viam.logging import getLogger
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.services.generic import Generic
from viam.services.motion import Motion
from viam.utils import ValueTypes, struct_to_dict

from .config import Settings
from .cycle import PickPlaceCycle
from .vision_geometry import CUBE_SELECTORS

IMPLEMENTATION = 'pickplace-v1-open-loop-suction-press'
LOGGER = getLogger(__name__)
API_VERSION = 1
CYCLE_ACTIONS = {'pick_place', 'run', 'box_once', 'ring_once', 'start'}  # start: pre-0.0.25 clients
MOTION_ACTIONS = CYCLE_ACTIONS | {'reset', 'recover_limits'}
STILL_ACTIONS = {'dry_run', 'measure_box', 'self_test'}  # background tasks that never move the arm


class PickPlaceService(Generic, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily('hackathons', 'armfarm19-vision-pickplace'), 'generic-service')

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        settings = Settings.from_attributes(struct_to_dict(config.attributes))
        return settings.dependencies(), []

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        self = super().new(config, dependencies)
        self.task = None
        self.state, self.error, self.last_result, self.request_id = 'idle', '', {}, ''
        self.used_requests = set()
        self._configure(config, dependencies)
        return self

    def _configure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        # Not Reconfigurable on purpose: a config change closes (stops) this instance and
        # builds a new one, so a running cycle is never re-pointed at new settings.
        s = Settings.from_attributes(struct_to_dict(config.attributes))
        lease = dependencies[Generic.get_resource_name(s.lease_service)] if s.lease_service else None
        self.settings = s
        self.cycle = PickPlaceCycle(
            s, dependencies[Arm.get_resource_name(s.arm)], dependencies[Gripper.get_resource_name(s.gripper)],
            dependencies[Camera.get_resource_name(s.camera)], dependencies[Motion.get_resource_name(s.motion)],
            lease, log=LOGGER.info)

    def busy(self):
        return self.task is not None and not self.task.done()

    def status(self):
        s, c = self.settings, self.cycle
        return {
            'implementation': IMPLEMENTATION, 'state': self.state, 'stage': c.stage, 'busy': self.busy(),
            'error': self.error, 'request_id': self.request_id, 'default_goal': s.default_goal,
            'goals': s.goals, 'action': getattr(self, 'action', ''),
            # Fields the code-3 web app (code3-web) checks before allowing a send.
            'available_goals': s.goals, 'calibrated': bool(s.calibration),
            'goal_details': [{'name': 'ring', 'type': 'ring'}]+[
                {'name': n, 'type': 'box', 'color': b.get('color')} for n, b in s.boxes.items()],
            'suction': {'mode': 'open_loop', 'holding_feedback_used': False, 'press_depth_mm': s.press_depth_mm,
                        'suction_dwell_s': s.suction_dwell_s, 'release_dwell_s': s.release_dwell_s},
            'speeds_deg_s': {'travel': s.travel_speed_deg_s, 'approach': s.approach_speed_deg_s, 'press': s.press_speed_deg_s},
            'joint_limit_margin_deg': s.joint_limit_margin_deg,
            'last_request': copy.deepcopy(getattr(self, 'last_request', {})),
            'plan': copy.deepcopy(c.plan_summary), 'last_result': copy.deepcopy(self.last_result),
            'events': copy.deepcopy(c.events[-30:]),
        }

    async def _work(self, action, args):
        goal, select = args.get('goal'), args.get('select', 'only')
        try:
            self.state, self.action = 'running', action
            if action in CYCLE_ACTIONS:
                self.last_result = await self.cycle.run(goal, select)
            elif action == 'reset':
                await self.cycle.reset()
                self.last_result = {'at_overview': True}
            elif action == 'recover_limits':
                self.last_result = await self.cycle.recover_limits()
            elif action == 'dry_run':
                self.last_result = await self.cycle.dry_run(goal, select)
            elif action == 'measure_box':
                self.last_result = await self.cycle.survey_box(args['name'])
            elif action == 'self_test':
                self.last_result = await self.cycle.self_test()
            self.state = 'complete'
        except asyncio.CancelledError:
            self.state, self.error = 'stopped', 'Stopped by command; inspect the workcell before the next run'
            raise
        except Exception as e:
            self.state, self.error = 'failed', f'{type(e).__name__}: {e}'
            LOGGER.error('%s failed at %s: %s', action, self.cycle.stage, e)

    def describe(self):
        """Machine-readable API description for UIs (web app, NL layer)."""
        s = self.settings
        return {
            'api_version': API_VERSION, 'implementation': IMPLEMENTATION,
            'pick': {'objects': ['cube'], 'select': list(CUBE_SELECTORS)+['{"index": n} (left to right)']},
            'place': {'goals': [{'name': 'ring', 'type': 'ring'}]+[
                {'name': n, 'type': 'box', 'color': b.get('color'), 'label': b.get('label', n)} for n, b in s.boxes.items()]},
            'default_goal': s.default_goal,
            'commands': {
                'pick_place': {'moves': True, 'example': {'action': 'pick_place', 'pick': {'object': 'cube', 'select': 'left'},
                                                          'place': {'goal': next(iter(s.boxes), 'ring')}, 'workcell_ready': True,
                                                          'request_id': 'unique-id-per-attempt'}},
                'dry_run': {'moves': False, 'example': {'action': 'dry_run', 'pick': {'select': 'only'}, 'place': {'type': 'ring'}}},
                'run': {'moves': True}, 'box_once': {'moves': True}, 'ring_once': {'moves': True},
                'start': {'moves': True, 'note': 'legacy alias of run, kept for older clients'},
                'reset': {'moves': True}, 'recover_limits': {'moves': True},
                'self_test': {'moves': False}, 'measure_box': {'moves': False}, 'status': {'moves': False},
                'describe': {'moves': False}, 'release': {'moves': False}, 'stop': {'moves': False},
            },
            'notes': 'Commands that move need "workcell_ready": true (a person confirmed the cell is clear). '
                     'Long commands return at once; poll {"action":"status"} until busy is false.',
        }

    def _arguments(self, action, command):
        """Normalise every command's target arguments to {goal, select, name}."""
        s, args = self.settings, {}
        if action == 'measure_box':
            name = command.get('name')
            if not isinstance(name, str) or not name or name == 'ring':
                raise ValueError('measure_box needs a "name" for the new box')
            return {'name': name}
        if action not in CYCLE_ACTIONS | {'dry_run'}:
            return args
        pick = command.get('pick', {})
        if not isinstance(pick, dict) or pick.get('object', 'cube') != 'cube':
            raise ValueError('pick.object must be "cube" (the only pickable object so far)')
        args['select'] = pick.get('select', command.get('select', 'only'))
        if args['select'] not in CUBE_SELECTORS and not isinstance(args['select'], dict):
            raise ValueError('pick.select must be one of '+', '.join(CUBE_SELECTORS)+' or {"index": n}')
        if action == 'ring_once':
            args['goal'] = 'ring'
        elif action == 'box_once':
            if not s.boxes:
                raise ValueError('No box is configured')
            args['goal'] = command.get('goal', next(iter(s.boxes)))
        elif 'place' in command:
            args['goal'] = s.resolve_place(command['place'])
        else:
            args['goal'] = command.get('goal', s.default_goal)
        if args['goal'] not in s.goals:
            raise ValueError('goal must be one of: '+', '.join(s.goals))
        return args

    async def do_command(self, command: Mapping[str, ValueTypes], *, timeout: Optional[float] = None, **kwargs) -> Mapping[str, ValueTypes]:
        action = command.get('action')
        if action == 'status':
            return self.status()
        if action == 'describe':
            return self.describe()
        if action == 'stop':
            if self.busy():
                self.task.cancel()
                await asyncio.gather(self.task, return_exceptions=True)
            await self.cycle.stop_arm()
            return self.status()
        if action == 'release':
            if self.busy():
                raise RuntimeError('code-2 is busy; stop it first')
            await self.cycle.suction_off(dwell=False)
            return self.status()
        if action not in MOTION_ACTIONS | STILL_ACTIONS:
            raise ValueError('Unknown action. Use describe, status, self_test, dry_run, measure_box, pick_place, run, '
                             'box_once, ring_once, reset, recover_limits, release, stop')
        if self.busy():
            return {**self.status(), 'accepted': False, 'already_running': True}
        if action in MOTION_ACTIONS and command.get('workcell_ready') is not True:
            raise ValueError('This command moves the arm; add "workcell_ready": true')
        request = command.get('request_id') or str(uuid.uuid4())
        if request in self.used_requests:
            return {**self.status(), 'accepted': False, 'duplicate_request': True}
        args = self._arguments(action, command)
        self.used_requests.add(request)
        self.request_id, self.error, self.last_result = request, '', {}
        self.last_request = {'action': action, **args}
        self.task = asyncio.create_task(self._work(action, args))
        return {**self.status(), 'accepted': True}

    async def close(self):
        if self.busy():
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)
            await self.cycle.stop_arm()
