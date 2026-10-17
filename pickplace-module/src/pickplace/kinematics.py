"""Forward/inverse kinematics from the arm's own URDF (as returned by Arm.GetKinematics).

Used to choose joint goals for free-space moves so the motion planner never picks an
IK branch that parks a joint on its limit. The xArm controller faults ("Joint Angle
Exceeds Limit") when the planner selects a joint exactly at its +/-360 deg bound.
Units: millimetres and degrees at the interface; radians internally.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

from .transforms import pose_matrix


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def _axis_angle(axis, angle):
    x, y, z = axis
    c, s, v = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    return np.array([[x*x*v+c, x*y*v-z*s, x*z*v+y*s],
                     [y*x*v+z*s, y*y*v+c, y*z*v-x*s],
                     [z*x*v-y*s, z*y*v+x*s, z*z*v+c]])


def _homogeneous(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def rotation_error_deg(a, b):
    a, b = np.asarray(a)[:3, :3], np.asarray(b)[:3, :3]
    return math.degrees(math.acos(np.clip((np.trace(a.T @ b)-1)/2, -1, 1)))


class ArmKinematics:
    """Serial chain world -> tool. tool_offset_mm is the gripper TCP along the flange Z."""

    def __init__(self, urdf, tool_offset_mm=126.0):
        root = ET.fromstring(urdf)
        by_parent = {}
        for j in root.findall('joint'):
            by_parent.setdefault(j.find('parent').get('link'), []).append(j)
        children = {j.find('child').get('link') for j in root.findall('joint')}
        roots = [l.get('name') for l in root.findall('link') if l.get('name') not in children]
        if len(roots) != 1:
            raise ValueError('URDF must have exactly one root link')
        self.chain = []  # (fixed transform, axis or None)
        self.limits_deg = []
        link = roots[0]
        while link in by_parent:
            if len(by_parent[link]) != 1:
                raise ValueError('Only serial URDF chains are supported')
            j = by_parent[link][0]
            origin = j.find('origin')
            xyz = [float(v)*1000 for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            fixed = _homogeneous(_rpy(*rpy), xyz)
            if j.get('type') in ('revolute', 'continuous'):
                axis = np.array([float(v) for v in j.find('axis').get('xyz').split()])
                axis /= np.linalg.norm(axis)
                limit = j.find('limit')
                self.limits_deg.append((math.degrees(float(limit.get('lower'))), math.degrees(float(limit.get('upper')))))
                self.chain.append((fixed, axis))
            elif j.get('type') == 'fixed':
                self.chain.append((fixed, None))
            else:
                raise ValueError('Unsupported URDF joint type '+str(j.get('type')))
            link = j.find('child').get('link')
        if len(self.limits_deg) != 6:
            raise ValueError('Expected a six-joint arm')
        self.tool = _homogeneous(np.eye(3), [0, 0, float(tool_offset_mm)])

    def fk(self, q_deg):
        """Tool pose in world as a 4x4 matrix (mm)."""
        if len(q_deg) != 6:
            raise ValueError('Six joint values required')
        T = np.eye(4)
        values = iter(q_deg)
        for fixed, axis in self.chain:
            T = T @ fixed
            if axis is not None:
                T = T @ _homogeneous(_axis_angle(axis, math.radians(next(values))), [0, 0, 0])
        return T @ self.tool

    def _jacobian(self, q_deg, eps=1e-4):
        base = self.fk(q_deg)
        J = np.zeros((6, 6))
        for i in range(6):
            q = list(q_deg)
            q[i] += math.degrees(eps)
            T = self.fk(q)
            J[:3, i] = (T[:3, 3]-base[:3, 3])/eps
            dR = T[:3, :3] @ base[:3, :3].T
            J[3:, i] = np.array([dR[2, 1]-dR[1, 2], dR[0, 2]-dR[2, 0], dR[1, 0]-dR[0, 1]])/(2*eps)
        return base, J

    def within_limits(self, q_deg, margin_deg=0.0):
        return all(lo+margin_deg <= v <= hi-margin_deg for v, (lo, hi) in zip(q_deg, self.limits_deg))

    def limit_clearance_deg(self, q_deg):
        """Smallest distance from any joint to its limit."""
        return min(min(v-lo, hi-v) for v, (lo, hi) in zip(q_deg, self.limits_deg))

    def ik(self, target, seed_deg, margin_deg=5.0, tolerance_mm=0.05, tolerance_deg=0.05):
        """Joint solution for a tool pose, nearest seed_deg, with margin_deg from every limit.

        target: pose dict (x,y,z,o_x,o_y,o_z,theta) or 4x4. Several seeds cover both wrist
        branches; full-turn joints are shifted by 360 deg toward the seed afterwards.
        """
        goal = np.array(pose_matrix(target)) if isinstance(target, dict) else np.asarray(target, float)
        seed = [float(v) for v in seed_deg]
        yaw = math.degrees(math.atan2(goal[1, 3], goal[0, 3]))
        seeds = [seed, [seed[0], seed[1], seed[2], seed[3]-180, -seed[4], seed[5]+180]]
        for j4, j5 in ((0, 90), (180, -90), (0, 45), (180, -45)):
            for j2, j3 in ((0, -90), (20, -120), (-20, -60)):
                seeds.append([yaw, j2, j3, j4, j5, seed[5]])
        solutions = []
        for start in seeds:
            q = self._solve(goal, start, margin_deg, tolerance_mm, tolerance_deg)
            if q is not None:
                q = self._normalize(q, seed, margin_deg)
                if q is not None:
                    solutions.append(q)
        if not solutions:
            raise ValueError(f'No IK solution keeps {margin_deg:g} deg from every joint limit')
        return min(solutions, key=lambda q: sum(abs(a-b) for a, b in zip(q, seed)))

    def _solve(self, goal, start, margin, tolerance_mm, tolerance_deg, iterations=200):
        """Damped least squares with the bounded joints (span < 360 deg) kept inside their margin."""
        q = np.array(start, float)
        bounded = [hi-lo < 360 for lo, hi in self.limits_deg]
        lo = np.array([l+margin if b else -1e9 for (l, _), b in zip(self.limits_deg, bounded)])
        hi = np.array([h-margin if b else 1e9 for (_, h), b in zip(self.limits_deg, bounded)])
        q = np.clip(q, lo, hi)
        for _ in range(iterations):
            T, J = self._jacobian(q)
            dp = goal[:3, 3]-T[:3, 3]
            dR = goal[:3, :3] @ T[:3, :3].T
            angle = math.acos(np.clip((np.trace(dR)-1)/2, -1, 1))
            w = np.array([dR[2, 1]-dR[1, 2], dR[0, 2]-dR[2, 0], dR[1, 0]-dR[0, 1]])
            w = w*(angle/(2*math.sin(angle))) if angle > 1e-9 else w/2
            if np.linalg.norm(dp) < tolerance_mm and math.degrees(angle) < tolerance_deg:
                return list(q)
            err = np.r_[dp, w*300.0]  # 1 rad of orientation error weighs like 300 mm
            Jw = J.copy()
            Jw[3:] *= 300.0
            step = Jw.T @ np.linalg.solve(Jw @ Jw.T+25.0*np.eye(6), err)
            q = np.clip(q+np.degrees(np.clip(step, -0.1, 0.1)), lo, hi)
        return None

    def _normalize(self, q, seed, margin):
        q = list(q)
        for i, (lo, hi) in enumerate(self.limits_deg):
            if hi-lo < 360:
                continue
            options = [q[i]+360*k for k in range(-3, 4) if lo+margin <= q[i]+360*k <= hi-margin]
            if not options:
                return None
            q[i] = min(options, key=lambda v: abs(v-seed[i]))
        return [float(v) for v in q] if self.within_limits(q, margin) else None
