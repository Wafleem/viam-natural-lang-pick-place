"""The math in GEOMETRY.md, checked: the grasp transform carries the cube exactly to its goal,
square to the box walls, for every cube orientation vision may report."""
import copy
import math

import numpy as np
import pytest

from pickplace.box import check_cube_fits
from pickplace.grasp_geometry import cube_top_frame, describe_grasp, place_tcp
from pickplace.task import compute_targets
from pickplace.transforms import pose_matrix
from pickplace.vision_geometry import downward_pose


def cube_at_release(targets):
    G = np.array(targets['grasp']['tcp_in_cube'])
    return np.array(pose_matrix(targets['release'])) @ np.linalg.inv(G)


def frame_yaw(T):
    return math.degrees(math.atan2(T[1, 0], T[0, 0]))


def mod90(deg):
    return (deg+45) % 90-45


@pytest.mark.parametrize('yaw', [-44.0, -20.0, -2.75, 0.0, 17.5, 44.0])
def test_cube_lands_on_goal_square_to_box(settings, observation, kin, yaw):
    obs = copy.deepcopy(observation)
    obs['cube']['world_yaw_deg'] = yaw
    t = compute_targets(settings, obs, 'box', kin, settings.overview_joints_deg)
    box = settings.boxes['box']
    C = cube_at_release(t)
    assert C[:3, 3] == pytest.approx(t['goal_top_world_mm'], abs=1e-6)
    assert mod90(frame_yaw(C)-box['world_yaw_deg']) == pytest.approx(0, abs=1e-6)
    assert min(check_cube_fits(box, settings.cube_size_mm, C[:3, 3], frame_yaw(C))) >= 0
    # the cup still points straight down at release
    assert np.array(pose_matrix(t['release']))[:3, 2] == pytest.approx([0, 0, -1], abs=1e-9)


def test_grasp_offset_turns_with_the_cube():
    """15 mm off-centre grip at pickup yaw 0, placed at 90: the tool offset rotates with the cube."""
    top = np.array([400.0, 200.0, 190.0])
    pick = downward_pose(top+[0, -15, 0], 0.0)
    G = describe_grasp(top, pick, 0.0, [200, 200])
    assert G['offset_in_cube_mm'][:2] == pytest.approx([0, -15])
    goal = np.array([300.0, -100.0, 200.0])
    release = place_tcp(goal, 90.0, G)
    assert [release['x'], release['y']] == pytest.approx([goal[0]+15, goal[1]], abs=1e-9)  # (0,-15) turned by +90
    assert np.allclose(np.array(pose_matrix(release)) @ np.linalg.inv(np.array(G['tcp_in_cube'])), cube_top_frame(goal, 90.0))


def test_current_box_clearance_is_documented(settings, observation, kin):
    """GEOMETRY.md section 5: 30.7 mm and 1.4 mm per side for the accepted box."""
    t = compute_targets(settings, observation, 'box', kin, settings.overview_joints_deg)
    gaps = check_cube_fits(settings.boxes['box'], settings.cube_size_mm, t['goal_top_world_mm'], t['place_yaw_deg'])
    assert gaps == pytest.approx([30.7, 1.4], abs=0.1)
    centred = {k: v for k, v in settings.boxes['box'].items() if k != 'placement_center_xy_mm'}
    assert check_cube_fits(centred, settings.cube_size_mm, centred['floor_center_world_mm'], centred['world_yaw_deg']) == pytest.approx([30.71, 3.28], abs=0.01)


def test_small_yaw_error_breaks_the_fit(settings):
    """Why yaw matters most: 5 deg off the walls adds ~17 mm of footprint."""
    box = settings.boxes['box']
    with pytest.raises(ValueError):
        check_cube_fits(box, settings.cube_size_mm, box['floor_center_world_mm'], box['world_yaw_deg']+5)
    check_cube_fits(box, settings.cube_size_mm, box['floor_center_world_mm'], box['world_yaw_deg']+90)


def test_pickup_orientation_uses_least_wrist_travel(settings, observation, kin):
    t = compute_targets(settings, observation, 'box', kin, settings.overview_joints_deg)
    travel = sum(abs(a-b) for a, b in zip(t['q_above_pick'], settings.overview_joints_deg))
    for k in (1, 2, 3):
        alt = downward_pose([t['contact'][a] for a in 'xyz'], t['cube_yaw_deg']+90*k)
        try:
            q = kin.ik({**alt, 'z': t['above_pick']['z']}, settings.overview_joints_deg, settings.joint_limit_margin_deg)
        except ValueError:
            continue
        assert travel <= sum(abs(a-b) for a, b in zip(q, settings.overview_joints_deg))+1e-6
