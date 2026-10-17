import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT/'tests'/'data'
sys.path.insert(0, str(ROOT/'src'))

from pickplace.config import Settings  # noqa: E402
from pickplace.kinematics import ArmKinematics  # noqa: E402
from pickplace.vision_geometry import analyze  # noqa: E402


@pytest.fixture(scope='session')
def attrs():
    return json.loads((ROOT/'config'/'code2-attributes.json').read_text())


@pytest.fixture(scope='session')
def settings(attrs):
    return Settings.from_attributes(attrs)


@pytest.fixture(scope='session')
def urdf():
    return (DATA/'xarm6_live.urdf').read_text()


@pytest.fixture(scope='session')
def kin(urdf):
    return ArmKinematics(urdf)


def load_frame(i):
    meta = json.loads((DATA/f'overview_{i}.json').read_text())
    bgr = cv2.imread(str(DATA/f'overview_{i}.png'))
    depth = np.load(DATA/f'overview_{i}_depth.npz')['depth'].astype(float)
    return meta, bgr, depth


@pytest.fixture(scope='session')
def observation(attrs):
    """A real overview observation (cube + ring) from the saved RGB-D frames."""
    meta, bgr, depth = load_frame(0)
    obs = analyze(bgr, depth, meta['properties'], meta['camera_pose_world'], attrs)
    obs.update(capture_time=meta['capture_time'], observation_joints_deg=meta['observation_joints_deg'])
    return obs


def interpolate(a, b, n=12):
    return [[x+(y-x)*i/(n-1) for x, y in zip(a, b)] for i in range(n)]


class FakeMotion:
    """Builtin-motion stand-in: 'plan' answers with joint/IK interpolation; records requests."""

    def __init__(self, kin, arm='arm'):
        self.kin, self.arm, self.requests = kin, arm, []
        self.poses = {}

    async def do_command(self, command, timeout=None):
        if 'plan' not in command:
            raise AssertionError('Only plan requests are expected; execution goes through the arm')
        request = json.loads(command['plan'])
        self.requests.append(request)
        start = [math.degrees(v) for v in request['extra']['start_state']['configuration'][self.arm]]
        if 'goal_state' in request['extra']:
            goal = [math.degrees(v) for v in request['extra']['goal_state']['configuration'][self.arm]]
        else:
            goal = self.kin.ik(request['destination']['pose'], start, margin_deg=0.5)
        return {'plan': [{self.arm: [math.radians(v) for v in q]} for q in interpolate(start, goal)]}

    async def get_pose(self, component, frame, timeout=None):
        return self.poses[(component, frame)]()
