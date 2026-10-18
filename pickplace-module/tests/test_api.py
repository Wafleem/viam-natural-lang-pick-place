"""Web/NL-facing API: pick selectors, place resolution, describe, extra cubes as obstacles."""
import asyncio
import copy
import json

import pytest

from pickplace.config import Settings
from pickplace.vision_geometry import select_cube


def cubes():
    return [{'pixel_hint': (900, 300), 'top_center_world_mm': [300, -100, 210]},
            {'pixel_hint': (200, 320), 'top_center_world_mm': [500, 200, 210]},
            {'pixel_hint': (550, 310), 'top_center_world_mm': [250, 50, 210]}]


@pytest.mark.parametrize('selector,expected', [('left', 200), ('right', 900), ('nearest', 550), ('farthest', 200),
                                               ({'index': 1}, 550)])
def test_select_cube(selector, expected):
    assert select_cube(cubes(), selector)['pixel_hint'][0] == expected


def test_select_only_requires_single_cube():
    with pytest.raises(ValueError, match='3 cubes visible'):
        select_cube(cubes(), 'only')
    assert select_cube(cubes()[:1], 'only')['pixel_hint'][0] == 900
    with pytest.raises(ValueError):
        select_cube(cubes(), {'index': 7})


def two_box_settings(attrs):
    box = {k: v for k, v in attrs['boxes'][0].items() if k != 'name'}
    blue = {**box, 'name': 'blue_box', 'color': 'Blue', 'floor_center_world_mm': [box['floor_center_world_mm'][0]+150, 330.0, box['floor_center_world_mm'][2]]}
    return Settings.from_attributes({**attrs, 'boxes': [{**box, 'name': 'box'}, blue]})


def test_resolve_place(attrs):
    s = two_box_settings(attrs)
    assert s.resolve_place('ring') == 'ring'
    assert s.resolve_place({'type': 'ring'}) == 'ring'
    assert s.resolve_place({'goal': 'blue_box'}) == 'blue_box'
    assert s.resolve_place({'color': 'blue'}) == 'blue_box'
    assert s.resolve_place({'color': 'orange'}) == 'box'
    assert s.resolve_place({'type': 'box'}) == 'box'
    for bad in ({'color': 'green'}, {'goal': 'shelf'}, {'type': 'bin'}, 5):
        with pytest.raises(ValueError):
            s.resolve_place(bad)


def make_service(settings, kin, urdf):
    from test_cycle import make_cycle
    from pickplace.service import PickPlaceService
    svc = PickPlaceService('code-2')
    svc.task, svc.state, svc.error, svc.last_result, svc.request_id, svc.used_requests = None, 'idle', '', {}, '', set()
    svc.settings = settings
    svc.cycle, *_ = make_cycle(settings, kin, urdf, settings.overview_joints_deg)
    return svc


def test_describe_is_json_and_lists_goals(attrs, kin, urdf):
    svc = make_service(two_box_settings(attrs), kin, urdf)
    d = asyncio.run(svc.do_command({'action': 'describe'}))
    json.dumps(d)
    assert [g['name'] for g in d['place']['goals']] == ['home', 'ring', 'box', 'blue_box', 'pink']
    assert next(g for g in d['place']['goals'] if g['name'] == 'pink')['entry'] == 'side'
    assert d['place']['goals'][3]['color'] == 'blue' and 'left' in d['pick']['select']


def test_pick_place_arguments(attrs, kin, urdf):
    svc = make_service(two_box_settings(attrs), kin, urdf)
    args = svc._arguments('pick_place', {'pick': {'object': 'cube', 'select': 'right'}, 'place': {'color': 'blue'}})
    assert args == {'select': 'right', 'goal': 'blue_box'}
    assert svc._arguments('box_once', {})['goal'] == 'box'
    assert svc._arguments('ring_once', {'pick': {'select': {'index': 0}}}) == {'select': {'index': 0}, 'goal': 'ring'}
    with pytest.raises(ValueError, match='pick.object'):
        svc._arguments('pick_place', {'pick': {'object': 'ball'}, 'place': 'ring'})
    with pytest.raises(ValueError, match='pick.select'):
        svc._arguments('pick_place', {'pick': {'select': 'middle'}, 'place': 'ring'})
    with pytest.raises(ValueError, match='workcell_ready'):
        asyncio.run(svc.do_command({'action': 'pick_place', 'place': 'ring'}))


def test_full_pick_place_runs_and_records_request(attrs, kin, urdf):
    svc = make_service(Settings.from_attributes(attrs), kin, urdf)

    async def go():
        reply = await svc.do_command({'action': 'pick_place', 'pick': {'select': 'only'}, 'place': {'type': 'box'},
                                      'workcell_ready': True, 'request_id': 'web-1'})
        await svc.task
        dup = await svc.do_command({'action': 'pick_place', 'place': 'box', 'workcell_ready': True, 'request_id': 'web-1'})
        return reply, dup
    reply, dup = asyncio.run(go())
    assert reply['accepted'] and svc.state == 'complete', svc.error
    assert svc.status()['last_request'] == {'action': 'pick_place', 'select': 'only', 'goal': 'box'}
    assert dup['duplicate_request']


def test_other_cubes_become_obstacles(settings, observation, kin):
    from pickplace.planning import scene_world
    from pickplace.task import compute_targets
    obs = copy.deepcopy(observation)
    extra = copy.deepcopy(obs['cube'])
    extra['top_center_world_mm'] = [extra['top_center_world_mm'][0]+250, extra['top_center_world_mm'][1], extra['top_center_world_mm'][2]]
    obs['cubes'] = [obs['cube'], extra]
    t = compute_targets(settings, obs, 'box', kin, settings.overview_joints_deg)
    labels = [g.label for g in scene_world(settings, obs, t, 'at_pick', 'box').obstacles[0].geometries]
    assert labels.count('other-cube-1') == 1 and 'other-cube-0' not in labels


def test_legacy_start_still_accepted(attrs, kin, urdf):
    svc = make_service(Settings.from_attributes(attrs), kin, urdf)
    assert svc._arguments('start', {'goal': 'ring'}) == {'select': 'only', 'goal': 'ring'}
    with pytest.raises(ValueError, match='workcell_ready'):
        asyncio.run(svc.do_command({'action': 'start', 'goal': 'ring'}))


def test_command_schema_lists_every_service_action():
    """The JSON schema handed to web apps / LLMs must not drift from what code-2 accepts."""
    import json
    from pathlib import Path
    from pickplace.service import MOTION_ACTIONS, STILL_ACTIONS
    schema = json.loads((Path(__file__).resolve().parents[2]/'nl-integration'/'command_schema.json').read_text())
    listed = set()
    for alt in schema['oneOf']:
        a = alt['properties']['action']
        listed |= set(a.get('enum', [])) | ({a['const']} if 'const' in a else set())
    legacy = {'run', 'box_once', 'ring_once', 'start'}  # aliases kept for older clients
    assert (MOTION_ACTIONS | STILL_ACTIONS) - legacy <= listed
