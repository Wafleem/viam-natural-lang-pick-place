"""One cube, three places (home, ring, box) and reset_cube: remembered placement, same grip spot,
correct rotation at every hand-off."""
import asyncio
import json
import math

import numpy as np
import pytest

from conftest import FakeMotion
from pickplace.memory import CubeMemory
from pickplace.task import compute_targets
from pickplace.transforms import pose_matrix
from test_cycle import make_cycle


def cube_frame_from_tool(pose, tcp_in_cube):
    return np.array(pose_matrix(pose)) @ np.linalg.inv(np.array(tcp_in_cube))


def yaw_of(T):
    return math.degrees(math.atan2(T[1, 0], T[0, 0]))


def mod90(a):
    return (a+45) % 90-45


def test_goals_are_home_ring_box(settings):
    assert settings.goals == ['home', 'ring', 'box']
    assert settings.resolve_place({'type': 'home'}) == 'home' and settings.resolve_place({'type': 'table'}) == 'home'


def test_box_then_reset_cube_regrips_same_spot(settings, kin, urdf, tmp_path):
    cycle, arm, gripper, motion, _ = make_cycle(settings, kin, urdf, settings.overview_joints_deg)
    cycle.memory = CubeMemory(tmp_path)

    async def seen_ok(goal, remembered=None):  # the fake camera never changes; pretend the pickup worked
        return {'cube_left_pick_spot': True, 'detail': 'simulated'}
    cycle.verify_outcome = seen_ok
    asyncio.run(cycle.run('box'))
    loc = cycle.memory.location
    placed = cycle.targets
    assert loc['place'] == 'box' and loc['top_world_mm'] == pytest.approx(placed['goal_top_world_mm'])
    # The remembered cube frame is exactly where the release put it.
    C_release = cube_frame_from_tool(placed['release'], loc['tcp_in_cube'])
    assert C_release[:3, 3] == pytest.approx(loc['top_world_mm'], abs=1e-6)
    assert mod90(yaw_of(C_release)-loc['yaw_deg']) == pytest.approx(0, abs=1e-6)

    # reset_cube: pick from the box with the same grip, put it on the home spot.
    arm.moves.clear()
    result = asyncio.run(cycle.reset_cube())
    t = cycle.targets
    assert t['source_place'] == 'box' and t['goal'] == 'home'
    # same cup spot on the cube as when it was released into the box
    assert {k: t['contact'][k] for k in 'xy'} == pytest.approx({k: placed['release'][k] for k in 'xy'}, abs=1e-6)
    assert t['contact']['theta'] == pytest.approx(placed['release']['theta'], abs=1e-6)
    # pressed 10 mm into the cube top, lifted clear of the rim before moving sideways
    assert t['press']['z'] == pytest.approx(loc['top_world_mm'][2]-settings.press_depth_mm)
    box = settings.boxes['box']
    assert t['above_pick']['z']-settings.cube_size_mm == pytest.approx(box['rim_z_mm']+settings.box_entry_clearance_mm)
    # seated square on the home spot at table height
    C_home = cube_frame_from_tool(t['release'], t['grasp']['tcp_in_cube'])
    assert C_home[:2, 3] == pytest.approx(settings.home['xy_mm'], abs=1e-6)
    assert C_home[2, 3] == pytest.approx(settings.calibration['pickup_tcp_z_mm'], abs=1e-6)
    assert mod90(yaw_of(C_home)-settings.home['yaw_deg']) == pytest.approx(0, abs=1e-6)
    assert result['cube_location']['place'] == 'home'
    # pick-from-box planning allowed the cube to touch the box floor, not the table
    press = motion.requests[-6]  # approach, descend, press, lift, transfer, place, retract, return
    allows = {(a['frame1'], a['frame2']) for a in press['constraints']['collision_specification'][0]['allows']}
    assert ('vision-cube', 'goal-box-floor') in allows and ('vision-cube', 'vision-table') not in allows
    assert gripper.calls[-2:] == ['grab', 'open']

    # Already home: nothing moves.
    arm.moves.clear()
    assert asyncio.run(cycle.reset_cube())['moved'] is False and arm.moves == []


def test_targets_are_deterministic(settings, observation, kin):
    a = compute_targets(settings, observation, 'box', kin, settings.overview_joints_deg)
    b = compute_targets(settings, observation, 'box', kin, settings.overview_joints_deg)
    assert json.dumps(a, sort_keys=True, default=float) == json.dumps(b, sort_keys=True, default=float)
    # grip spot: grasp_toward_image_top_mm from the top-face centre, toward image-top, every time
    assert a['grasp']['offset_in_cube_mm'][:2] == pytest.approx([0, -settings.grasp_toward_image_top_mm], abs=1e-6)


def test_memory_persists_and_unknown_blocks_reset(settings, kin, urdf, tmp_path):
    m = CubeMemory(tmp_path)
    m.set('box', [1, 2, 3], 4.0, np.eye(4).tolist(), how='test')
    assert CubeMemory(tmp_path).location['place'] == 'box'
    assert CubeMemory(tmp_path).pickable_source()['top_world_mm'] == [1, 2, 3]
    cycle, arm, *_ = make_cycle(settings, kin, urdf, settings.overview_joints_deg)
    cycle.memory = CubeMemory(tmp_path/'other')
    from pickplace.cycle import CycleError
    with pytest.raises(CycleError, match='unknown'):
        asyncio.run(cycle.reset_cube())
    assert arm.moves == []


def test_set_cube_location_for_a_box(settings, kin, urdf, tmp_path):
    cycle, *_ = make_cycle(settings, kin, urdf, settings.overview_joints_deg)
    cycle.memory = CubeMemory(tmp_path)
    loc = cycle.set_location('box', settings.goals)
    src = cycle.memory.pickable_source()
    assert src['place'] == 'box' and src['top_world_mm'][2] == pytest.approx(settings.boxes['box']['floor_center_world_mm'][2]+200)
    # assumed grip is the face centre, tool down
    C = cube_frame_from_tool(__import__('pickplace.grasp_geometry', fromlist=['place_tcp']).place_tcp(np.array(src['top_world_mm']), src['yaw_deg'], {'tcp_in_cube': src['tcp_in_cube']}), src['tcp_in_cube'])
    assert C[:3, 3] == pytest.approx(src['top_world_mm'], abs=1e-6)
    assert cycle.set_location('table', settings.goals)['place'] == 'table' and cycle.memory.pickable_source() is None
    with pytest.raises(Exception):
        cycle.set_location('shelf', settings.goals)
    assert loc['how'].startswith('set by operator')


def test_failure_after_contact_forgets_location(settings, kin, urdf, tmp_path):
    cycle, arm, *_ = make_cycle(settings, kin, urdf, settings.overview_joints_deg)
    cycle.memory = CubeMemory(tmp_path)
    cycle.memory.set('table', how='seen')
    real = arm.move_through_joint_positions
    calls = {'n': 0}

    async def failing(positions, options=None, timeout=None):
        calls['n'] += 1
        if calls['n'] == 4:  # approach, descend, press, then fail on lift
            raise RuntimeError('xArm: Joint Angle Exceeds Limit')
        return await real(positions, options, timeout)
    arm.move_through_joint_positions = failing
    with pytest.raises(RuntimeError):
        asyncio.run(cycle.run('box'))
    assert cycle.memory.location['place'] == 'unknown'
    # A failure before any contact (here: planning) keeps what we knew.
    cycle2, arm2, *_ = make_cycle(settings, kin, urdf, settings.overview_joints_deg)
    cycle2.memory = CubeMemory(tmp_path/'b')
    cycle2.memory.set('table', how='seen')
    cycle2.motion.do_command = lambda *a, **k: (_ for _ in ()).throw(RuntimeError('planner down'))
    with pytest.raises(Exception):
        asyncio.run(cycle2.run('box'))
    assert cycle2.memory.location['place'] == 'table'


def test_fixed_ring_spot_needs_no_vision(attrs, observation, kin):
    from pickplace.config import Settings
    s = Settings.from_attributes({**attrs, 'ring': {'xy_mm': [300, -470]}})
    obs = {k: v for k, v in observation.items() if k != 'ring'}
    t = compute_targets(s, obs, 'ring', kin, s.overview_joints_deg)
    C = cube_frame_from_tool(t['release'], t['grasp']['tcp_in_cube'])
    assert C[:2, 3] == pytest.approx([300, -470], abs=1e-6) and C[2, 3] == pytest.approx(s.calibration['place_tcp_z_mm'])
    from pickplace.planning import scene_world
    labels = {g.label for g in scene_world(s, obs, t, 'placed', 'ring').obstacles[0].geometries}
    assert {'vision-ring', 'vision-ring-45', 'vision-ring-90', 'vision-ring-135'} <= labels


def test_missed_seal_or_drop_retries_then_gives_up(settings, kin, urdf, tmp_path):
    cycle, arm, gripper, *_ = make_cycle(settings, kin, urdf, settings.overview_joints_deg)
    cycle.memory = CubeMemory(tmp_path)
    dropped = []

    def first_lift_drops(on):  # seals every time; the first check after a lift shows it gone
        if on and cycle.events[-1]['stage'] == 'lift' and not dropped:
            dropped.append(True)
            return False
        return on
    gripper.script = first_lift_drops

    async def seen_ok(goal, remembered=None):
        return {'cube_left_pick_spot': True, 'detail': 'simulated'}
    cycle.verify_outcome = seen_ok
    asyncio.run(cycle.run('box'))
    names = [e['stage'] for e in cycle.events]
    assert names.count('press') == 2 and 'retrying_pick' in names and names[-1] == 'complete'
    assert [e['held'] for e in cycle.events if e['stage'] == 'hold_check'] == [False, True]

    # Never seals: no lift at all, pick_retries+1 presses, then a clear error.
    cycle2, arm2, gripper2, *_ = make_cycle(settings, kin, urdf, settings.overview_joints_deg)
    cycle2.memory = CubeMemory(tmp_path/'b')
    gripper2.script = lambda on: False
    from pickplace.cycle import CycleError
    with pytest.raises(CycleError, match='No grip'):
        asyncio.run(cycle2.run('box'))
    names = [e['stage'] for e in cycle2.events]
    assert names.count('press') == settings.pick_retries+1 and 'lift' not in names
