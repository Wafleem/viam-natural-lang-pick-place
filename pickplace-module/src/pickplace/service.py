"""Viam generic service `code-2`: DoCommand front end for PickPlaceCycle.

Commands (JSON in the Viam app CONTROL tab -> code-2 -> DoCommand):
  {"action": "status"}                                         read only
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

IMPLEMENTATION = 'pickplace-v1-open-loop-suction-press'
LOGGER = getLogger(__name__)
MOTION_ACTIONS = {'run', 'box_once', 'ring_once', 'reset', 'recover_limits'}
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
            'goals': s.goals,
            'suction': {'mode': 'open_loop', 'holding_feedback_used': False, 'press_depth_mm': s.press_depth_mm,
                        'suction_dwell_s': s.suction_dwell_s, 'release_dwell_s': s.release_dwell_s},
            'speeds_deg_s': {'travel': s.travel_speed_deg_s, 'approach': s.approach_speed_deg_s, 'press': s.press_speed_deg_s},
            'joint_limit_margin_deg': s.joint_limit_margin_deg,
            'plan': copy.deepcopy(c.plan_summary), 'last_result': copy.deepcopy(self.last_result),
            'events': copy.deepcopy(c.events[-30:]),
        }

    async def _work(self, action, goal):
        try:
            self.state = 'running '+action
            if action in ('run', 'box_once', 'ring_once'):
                self.last_result = await self.cycle.run(goal)
            elif action == 'reset':
                await self.cycle.reset()
                self.last_result = {'at_overview': True}
            elif action == 'recover_limits':
                self.last_result = await self.cycle.recover_limits()
            elif action == 'dry_run':
                self.last_result = await self.cycle.dry_run(goal)
            elif action == 'measure_box':
                self.last_result = await self.cycle.survey_box(goal)
            elif action == 'self_test':
                self.last_result = await self.cycle.self_test()
            self.state = 'complete'
        except asyncio.CancelledError:
            self.state, self.error = 'stopped', 'Stopped by command; inspect the workcell before the next run'
            raise
        except Exception as e:
            self.state, self.error = 'failed', f'{type(e).__name__}: {e}'
            LOGGER.error('%s failed at %s: %s', action, self.cycle.stage, e)

    async def do_command(self, command: Mapping[str, ValueTypes], *, timeout: Optional[float] = None, **kwargs) -> Mapping[str, ValueTypes]:
        action = command.get('action')
        if action == 'status':
            return self.status()
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
            raise ValueError('Unknown action. Use status, self_test, dry_run, measure_box, run, box_once, ring_once, reset, recover_limits, release, stop')
        if self.busy():
            return {**self.status(), 'accepted': False, 'already_running': True}
        if action in MOTION_ACTIONS and command.get('workcell_ready') is not True:
            raise ValueError('This command moves the arm; add "workcell_ready": true')
        request = command.get('request_id') or str(uuid.uuid4())
        if request in self.used_requests:
            return {**self.status(), 'accepted': False, 'duplicate_request': True}
        goal = command.get('goal', self.settings.default_goal)
        if action == 'measure_box':
            goal = command.get('name')
            if not isinstance(goal, str) or not goal or goal == 'ring':
                raise ValueError('measure_box needs a "name" for the new box')
        elif action == 'self_test':
            goal = None
        elif action == 'ring_once':
            goal = 'ring'
        elif action == 'box_once':
            if not self.settings.boxes:
                raise ValueError('No box is configured')
            goal = command.get('goal', next(iter(self.settings.boxes)))
        if action in ('run', 'box_once', 'ring_once', 'dry_run') and goal not in self.settings.goals:
            raise ValueError('goal must be one of: '+', '.join(self.settings.goals))
        self.used_requests.add(request)
        self.request_id, self.error, self.last_result = request, '', {}
        self.task = asyncio.create_task(self._work(action, goal))
        return {**self.status(), 'accepted': True}

    async def close(self):
        if self.busy():
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)
            await self.cycle.stop_arm()
