"""Viam generic service `code-2`: DoCommand front end for PickPlaceCycle.

Commands (JSON in the Viam app CONTROL tab -> code-2 -> DoCommand):
  {"action": "status"}                                         read only
  {"action": "dry_run", "goal": "box"}                         detect + plan, no motion (at overview)
  {"action": "run", "goal": "ring"|"box", "workcell_ready": true}   one full cycle (MOVES)
  {"action": "box_once", "workcell_ready": true}               alias: run goal=box
  {"action": "ring_once", "workcell_ready": true}              alias: run goal=ring
  {"action": "reset", "workcell_ready": true}                  suction off, go to overview (MOVES)
  {"action": "recover_limits", "workcell_ready": true}         back off a joint stuck at its limit (MOVES)
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
            'box_configured': bool(s.box),
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
        if action not in MOTION_ACTIONS | {'dry_run'}:
            raise ValueError('Unknown action. Use status, dry_run, run, box_once, ring_once, reset, recover_limits, release, stop')
        if self.busy():
            return {**self.status(), 'accepted': False, 'already_running': True}
        if action in MOTION_ACTIONS and command.get('workcell_ready') is not True:
            raise ValueError('This command moves the arm; add "workcell_ready": true')
        request = command.get('request_id') or str(uuid.uuid4())
        if request in self.used_requests:
            return {**self.status(), 'accepted': False, 'duplicate_request': True}
        goal = {'box_once': 'box', 'ring_once': 'ring'}.get(action, command.get('goal', self.settings.default_goal))
        if goal not in ('ring', 'box'):
            raise ValueError('goal must be ring or box')
        if goal == 'box' and not self.settings.box:
            raise ValueError('Box goal needs the accepted box reference in config')
        self.used_requests.add(request)
        self.request_id, self.error, self.last_result = request, '', {}
        self.task = asyncio.create_task(self._work(action, goal))
        return {**self.status(), 'accepted': True}

    async def close(self):
        if self.busy():
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)
            await self.cycle.stop_arm()
