"""Accepted orange Post-it box reference: fit check and collision cavity.

The box pose/aperture/floor were measured once and accepted (see DEPLOYMENT.md).
They are reused as configured; if the box is moved, re-measure and update config.
"""
import math

import numpy as np


def _rotation(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def placement_xy(box):
    return np.asarray(box.get('placement_center_xy_mm', box['floor_center_world_mm'][:2]), float)


def check_cube_fits(box, cube_size, center_xy, world_yaw):
    """Cube footprint inside the opening at this position/yaw. Returns per-axis gap (mm)."""
    dims = np.asarray(box['inner_dimensions_mm'], float)
    delta = (np.asarray(center_xy[:2], float)-np.asarray(box['floor_center_world_mm'][:2], float)) @ _rotation(box['world_yaw_deg'])
    theta = math.radians(world_yaw-box['world_yaw_deg'])
    extent = cube_size*(abs(math.cos(theta))+abs(math.sin(theta)))
    remaining = dims/2-np.abs(delta)-extent/2
    if min(remaining) < 0:
        raise ValueError(f'Cube does not fit the box opening (gaps {remaining.round(1).tolist()} mm)')
    return remaining.tolist()


def cavity_geometry(box, name='box', wall_mm=30.0):
    """[(label, center_xyz, dims_xyz, yaw_deg)]: a floor slab and four wall prisms.

    Labels are floor_label(name) and goal-<name>-wall-<i>, unique per box."""
    c = np.asarray(box['floor_center_world_mm'], float)
    R = _rotation(box['world_yaw_deg'])
    width, length = box['inner_dimensions_mm']
    height = box['rim_z_mm']-c[2]
    result = [(floor_label(name), [*c[:2], c[2]-10], [width+2*wall_mm, length+2*wall_mm, 20], box['world_yaw_deg'])]
    walls = [([-width/2-wall_mm/2, 0], [wall_mm, length+2*wall_mm, height]),
             ([width/2+wall_mm/2, 0], [wall_mm, length+2*wall_mm, height]),
             ([0, -length/2-wall_mm/2], [width, wall_mm, height]),
             ([0, length/2+wall_mm/2], [width, wall_mm, height])]
    for i, (offset, dims) in enumerate(walls):
        xy = c[:2]+R @ offset
        result.append((f'goal-{name}-wall-{i}', [*xy, c[2]+height/2], dims, box['world_yaw_deg']))
    return result


def floor_label(name):
    return f'goal-{name}-floor'
