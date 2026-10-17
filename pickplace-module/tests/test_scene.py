"""dynamic-scene: tracked boxes follow the camera; dimensions and heights stay accepted."""
import asyncio

import numpy as np
import pytest

from pickplace import scene
from pickplace.config import Settings


def tracked_settings(attrs, mode='cached', track=True):
    box = {**attrs['boxes'][0], 'track': track}
    return Settings.from_attributes({**attrs, 'boxes': [box], 'box_tracking': mode})


def measurement(box, dx=0.0, dy=0.0, dyaw=0.0, dims=(250.0, 200.0)):
    c = box['floor_center_world_mm']
    return {'floor_center_world_mm': [c[0]+dx, c[1]+dy, c[2]+7], 'world_yaw_deg': box['world_yaw_deg']+dyaw,
            'inner_dimensions_mm': list(dims), 'rim_z_mm': box['rim_z_mm']+9}


def test_relocate_moves_pose_only(attrs):
    s = tracked_settings(attrs)
    box = s.boxes['box']
    moved, reason = scene.relocate(box, [measurement(box, 40, -25, 3) for _ in range(3)])
    assert reason is None
    assert moved['floor_center_world_mm'][:2] == pytest.approx([box['floor_center_world_mm'][0]+40, box['floor_center_world_mm'][1]-25])
    assert moved['floor_center_world_mm'][2] == box['floor_center_world_mm'][2]
    assert moved['inner_dimensions_mm'] == box['inner_dimensions_mm'] and moved['rim_z_mm'] == box['rim_z_mm']
    assert moved['world_yaw_deg'] == pytest.approx(box['world_yaw_deg']+3)
    # the placement point keeps its offset from the floor centre, rotated with the box
    old = np.subtract(box['placement_center_xy_mm'], box['floor_center_world_mm'][:2])
    new = np.subtract(moved['placement_center_xy_mm'], moved['floor_center_world_mm'][:2])
    assert np.linalg.norm(new) == pytest.approx(np.linalg.norm(old))


def test_relocate_needs_agreeing_frames(attrs):
    box = tracked_settings(attrs).boxes['box']
    assert scene.relocate(box, [measurement(box), {'error': 'clipped'}, {'error': 'clipped'}])[0] is None
    noisy = [measurement(box, 0), measurement(box, 10), measurement(box, 0)]
    assert 'disagree' in scene.relocate(box, noisy)[1]
    far = [measurement(box, 600) for _ in range(3)]
    assert scene.relocate(box, far)[0] is None


def test_tracking_selection(attrs):
    assert scene.tracked_names(tracked_settings(attrs, 'cached', False)) == []
    assert scene.tracked_names(tracked_settings(attrs, 'cached', True)) == ['box']
    assert scene.tracked_names(tracked_settings(attrs, 'live', False)) == ['box']
    with pytest.raises(ValueError):
        Settings.from_attributes({**attrs, 'box_tracking': 'sometimes'})


def test_track_boxes_and_targets_follow(attrs, observation, kin, monkeypatch):
    from pickplace.task import compute_targets
    s = tracked_settings(attrs)
    box = s.boxes['box']
    monkeypatch.setattr(scene, 'measure_frames', lambda *a: [measurement(box, 30, 0) for _ in range(5)])
    moved, report = scene.track_boxes(s, [None]*5, [observation]*5)
    assert report['boxes']['box']['updated'] and report['boxes']['box']['shift_mm'] == pytest.approx(30, abs=.1)
    before = compute_targets(s, observation, 'box', kin, s.overview_joints_deg)['goal_top_world_mm']
    after = compute_targets(moved, observation, 'box', kin, s.overview_joints_deg)['goal_top_world_mm']
    assert after[0]-before[0] == pytest.approx(30, abs=.01) and s.boxes['box'] is box


def test_cycle_keeps_unseen_box(attrs, kin, urdf):
    from test_cycle import make_cycle
    s = tracked_settings(attrs, 'live')
    cycle, *_ = make_cycle(s, kin, urdf, s.overview_joints_deg)
    asyncio.run(cycle.run('box'))
    report = cycle.scene_report['boxes']['box']
    assert report['updated'] is False and 'need 3' in report['reason']
    assert cycle.plan_summary['scene']['mode'] == 'live'
