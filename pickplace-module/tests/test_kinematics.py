import json
import math

import numpy as np
import pytest

from conftest import DATA
from pickplace.transforms import pose_matrix

OVERVIEW = [-4.884027084421052, -33.22272655695361, -78.57937979708325, 187.7903548517328, -94.99998784487263, -190.8119758990331]
# Above-pickup goal from the 0.0.24 run that faulted: the planner chose J4 = 360.000 (the URDF bound).
FAILED_ABOVE_PICKUP = {'x': 378.9218632337712, 'y': 241.08222533258754, 'z': 430.0, 'o_x': 0.0, 'o_y': 0.0, 'o_z': -1.0, 'theta': -1.6253886194490406}


def test_fk_matches_live_frame_system(kin):
    live = json.loads((DATA/'live_pose_failed_lift.json').read_text())
    T = kin.fk(live['joints_deg'])
    G = np.array(pose_matrix(live['poses']['Vgripper']))
    assert np.linalg.norm(T[:3, 3]-G[:3, 3]) < 1e-6
    assert np.allclose(T[:3, :3], G[:3, :3], atol=1e-6)


def test_urdf_limits_are_full_turn_for_j4(kin):
    assert kin.limits_deg[3] == pytest.approx((-360, 360))


def test_ik_keeps_wrist_away_from_limit(kin):
    q = kin.ik(FAILED_ABOVE_PICKUP, OVERVIEW, margin_deg=5)
    assert abs(q[3]) < 1 and kin.limit_clearance_deg(q) >= 5
    T = kin.fk(q)
    assert np.linalg.norm(T[:3, 3]-np.array([FAILED_ABOVE_PICKUP[k] for k in 'xyz'])) < .1


def test_ik_full_turn_joints_shift_toward_seed(kin):
    q = kin.ik(FAILED_ABOVE_PICKUP, OVERVIEW)
    alternatives = [q[5]+360*k for k in (-1, 1) if abs(q[5]+360*k) <= 355]
    assert all(abs(q[5]-OVERVIEW[5]) <= abs(a-OVERVIEW[5]) for a in alternatives)


def test_ik_rejects_unreachable_pose(kin):
    with pytest.raises(ValueError):
        kin.ik({'x': 2000, 'y': 0, 'z': 300, 'o_z': -1, 'theta': 0}, OVERVIEW)


def test_ik_roundtrip_random_reachable_poses(kin):
    rng = np.random.default_rng(3)
    for _ in range(5):
        q = [rng.uniform(-60, 60), rng.uniform(-40, 20), rng.uniform(-120, -40), rng.uniform(-90, 90), rng.uniform(20, 120), rng.uniform(-150, 150)]
        T = kin.fk(q)
        sol = kin.ik(T, q)
        assert np.linalg.norm(kin.fk(sol)[:3, 3]-T[:3, 3]) < .1
        assert math.degrees(math.acos(np.clip((np.trace(kin.fk(sol)[:3, :3].T @ T[:3, :3])-1)/2, -1, 1))) < .1
