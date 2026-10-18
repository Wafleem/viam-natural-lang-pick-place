import json

import numpy as np
import pytest

from conftest import DATA, load_frame
from pickplace.box import cavity_geometry, check_cube_fits
from pickplace.config import Settings
from pickplace.task import compute_targets
from pickplace.vision_geometry import analyze, consensus_observation


def test_old_attributes_still_parse():
    s = Settings.from_attributes(json.loads((DATA/'attributes-0.0.24.json').read_text()))
    assert list(s.boxes) == ['box'] and s.boxes['box']['rim_z_mm'] == pytest.approx(129.99776389646297)
    assert s.press_depth_mm == 10


@pytest.mark.parametrize('bad', [{'press_depth_mm': 25}, {'press_depth_mm': -1}, {'default_goal': 'shelf'},
                                 {'aligned_depth_enabled': False}, {'travel_speed_deg_s': 150}])
def test_config_rejects_bad_values(attrs, bad):
    with pytest.raises(ValueError):
        Settings.from_attributes({**attrs, **bad})


def test_perception_on_saved_overview(attrs):
    frames = []
    for i in range(3):
        meta, bgr, depth = load_frame(i)
        obs = analyze(bgr, depth, meta['properties'], meta['camera_pose_world'], attrs)
        assert [e for e in obs['issues'] if not e.startswith('Depth/world tabletop')] == []
        assert 180 < obs['cube']['observed_height_mm'] < 260 and 'ring' in obs
        frames.append(obs)
    centers = np.array([f['cube']['top_center_world_mm'] for f in frames])
    assert np.ptp(centers, axis=0).max() < 5


def test_ring_optional_for_box(attrs):
    meta, bgr, depth = load_frame(0)
    ring_px = None
    from pickplace.find_cube_ring import find_objects
    _, rings = find_objects(bgr)
    x, y, w, h = rings[0]['bbox']
    bgr = bgr.copy()
    bgr[max(0, y-20):y+h+20, max(0, x-20):x+w+20] = 235  # paint the ring out
    obs = analyze(bgr, depth, meta['properties'], meta['camera_pose_world'], attrs, require_ring=False)
    assert 'cube' in obs and 'ring' not in obs and ring_px is None
    obs_ring = analyze(bgr, depth, meta['properties'], meta['camera_pose_world'], attrs, require_ring=True)
    assert 'Need exactly one full ring in view' in obs_ring['issues']


def test_consensus_requires_consistent_ring(observation):
    frames = []
    for i in range(5):
        o = json.loads(json.dumps(observation))
        o['capture_time'] = f't{i}'
        frames.append(o)
    assert consensus_observation(frames)['detection_quality']['agreeing_frames'] == 5
    del frames[2]['ring']
    with pytest.raises(ValueError):
        consensus_observation(frames)


@pytest.mark.parametrize('goal', ['box', 'ring'])
def test_targets_press_ten_mm_and_keep_margins(settings, observation, kin, goal):
    t = compute_targets(settings, observation, goal, kin, settings.overview_joints_deg)
    contact_z = settings.calibration['pickup_tcp_z_mm']
    assert t['contact']['z'] == pytest.approx(contact_z)
    assert t['press']['z'] == pytest.approx(contact_z-settings.press_depth_mm)
    assert t['pre_contact']['z'] == pytest.approx(contact_z+20)
    assert {k: t['press'][k] for k in 'xy'} == {k: t['contact'][k] for k in 'xy'}
    for q in (t['q_above_pick'], t['q_above_place']):
        assert kin.limit_clearance_deg(q) >= settings.joint_limit_margin_deg
        assert abs(q[3]) <= 360-settings.joint_limit_margin_deg
    # Box: both ends on the taught wrist branch (J4 ~ 180), like the demonstrated cycle.
    if goal == 'box':
        assert all(150 < q[3] < 210 for q in (t['q_above_pick'], t['q_above_place']))
    # Release keeps the nominal (unpressed) grasp height: cube bottom at the goal surface.
    assert t['release']['z'] == pytest.approx(t['goal_top_world_mm'][2], abs=1e-6)


def test_box_goal_centres_cube_and_fits(settings, observation, kin):
    t = compute_targets(settings, observation, 'box', kin, settings.overview_joints_deg)
    box = settings.boxes['box']
    assert t['goal_top_world_mm'][:2] == pytest.approx(box['placement_center_xy_mm'])
    assert t['goal_top_world_mm'][2] == pytest.approx(box['floor_center_world_mm'][2]+settings.cube_size_mm)
    gaps = check_cube_fits(box, settings.cube_size_mm, t['goal_top_world_mm'], t['place_yaw_deg'])
    assert min(gaps) > 0
    assert (t['place_yaw_deg']-box['world_yaw_deg']) % 90 == pytest.approx(0, abs=1e-9) or \
        (t['place_yaw_deg']-box['world_yaw_deg']) % 90 == pytest.approx(90, abs=1e-9)
    # Above the box the cube bottom clears the rim by box_entry_clearance_mm before entering.
    assert t['above_place']['z']-settings.cube_size_mm == pytest.approx(box['rim_z_mm']+settings.box_entry_clearance_mm)
    assert t['above_pick']['z'] == pytest.approx(t['contact']['z']+settings.clearance_mm)


def test_box_fit_rejects_wall_contact(settings):
    box = settings.boxes['box']
    with pytest.raises(ValueError):
        check_cube_fits(box, settings.cube_size_mm, [box['floor_center_world_mm'][0], box['floor_center_world_mm'][1]+10], box['world_yaw_deg'])
    with pytest.raises(ValueError):
        check_cube_fits(box, settings.cube_size_mm, box['floor_center_world_mm'], box['world_yaw_deg']+20)
    assert [g[0] for g in cavity_geometry(box, 'box')][:2] == ['goal-box-floor', 'goal-box-wall-0']


def test_multiple_named_boxes(attrs, observation, kin):
    box = {k: v for k, v in attrs['boxes'][0].items() if k != 'name'}
    second = {**box, 'name': 'box2', 'floor_center_world_mm': [box['floor_center_world_mm'][0]+150, 330.0, box['floor_center_world_mm'][2]],
              'placement_center_xy_mm': [box['floor_center_world_mm'][0]+150, 330.0], 'rim_z_mm': 160.0}
    s = Settings.from_attributes({**attrs, 'boxes': [{**box, 'name': 'box'}, second]})
    assert s.goals == ['home', 'ring', 'box', 'box2', *s.side_boxes]
    t = compute_targets(s, observation, 'box2', kin, s.overview_joints_deg)
    assert t['goal_top_world_mm'][:2] == pytest.approx(second['placement_center_xy_mm'])
    assert t['above_place']['z']-s.cube_size_mm == pytest.approx(160+s.box_entry_clearance_mm)  # its own rim
    from pickplace.planning import scene_world, allowed_contacts
    labels = {g.label for g in scene_world(s, observation, t, 'placed', 'box2').obstacles[0].geometries}
    assert {'goal-box-floor', 'goal-box2-floor', 'goal-box2-wall-3'} <= labels
    assert ('vision-cube', 'goal-box2-floor') in allowed_contacts(s, 'place', 'box2')


@pytest.mark.parametrize('bad', [[{'name': 'ring'}], [{'name': 'a'}, {'name': 'a'}], 'box'])
def test_bad_box_lists_rejected(attrs, bad):
    with pytest.raises(ValueError):
        Settings.from_attributes({**attrs, 'boxes': bad})


def test_box_survey_reproduces_accepted_opening():
    """The ported survey measures the accepted box's own source frame (box view, not overview)."""
    import cv2
    from conftest import DATA
    from pickplace.box_survey import measure_box
    meta = json.loads((DATA/'box_view_5.json').read_text())
    depth = np.load(DATA/'box_view_5_depth.npz')['depth'].astype(float)
    b = measure_box(cv2.imread(str(DATA/'box_view_5.png')), depth, meta['properties'], meta['camera_pose_world'])
    accepted = json.loads((DATA/'attributes-0.0.24.json').read_text())['surveyed_box']['box']
    assert np.allclose(b['inner_dimensions_mm'], accepted['inner_dimensions_mm'], atol=2.5)
    assert 120 < b['cavity_depth_mm'] < 140


def test_box_candidate_uses_overview_calibration(settings):
    from pickplace.box_survey import candidate_entry
    cal = settings.calibration
    raw = {'floor_center_world_mm': [300.0, -150.0, 30.0], 'inner_dimensions_mm': [261.0, 206.0], 'world_yaw_deg': 1.0, 'rim_z_mm': 160.0}
    table_z = 20.0
    c = candidate_entry('box2', raw, cal, table_z, settings.cube_size_mm)
    dz = cal['pickup_tcp_z_mm']-settings.cube_size_mm-table_z
    assert c['floor_center_world_mm'][2] == pytest.approx(30+dz) and c['rim_z_mm'] == pytest.approx(160+dz)
    assert c['world_yaw_deg'] == pytest.approx(1+cal['yaw_correction_deg'])
    assert list(Settings.from_attributes({'task_calibration': cal, 'aligned_depth_enabled': True, 'boxes': [c]}).boxes) == ['box2']
