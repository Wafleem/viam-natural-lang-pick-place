"""Explicit cube-to-TCP transform, shared by pickup and placement geometry."""
import math
import numpy as np
from .transforms import pose_matrix,matrix_pose

def cube_top_frame(center,yaw_deg):
    center=np.asarray(center,dtype=float)
    if center.shape!=(3,) or not np.all(np.isfinite(center)) or not math.isfinite(yaw_deg):
        raise ValueError('Finite measured cube center and yaw required')
    return np.array(pose_matrix({**dict(zip('xyz',center)),'o_z':1,'theta':float(yaw_deg)}))

def describe_grasp(cube_top_world,pickup_tcp_world,cube_yaw_deg,top_dimensions_mm):
    """Cube frame origin is its top-face center, Z up; dimensions are observed."""
    dims=np.asarray(top_dimensions_mm,dtype=float)
    if dims.shape!=(2,) or not np.all(np.isfinite(dims)) or min(dims)<=0:
        raise ValueError('Observed top-face dimensions required')
    C=cube_top_frame(cube_top_world,cube_yaw_deg)
    G=np.linalg.inv(C)@np.array(pose_matrix(pickup_tcp_world))
    if not np.all(np.isfinite(G)):raise ValueError('Invalid grasp transform')
    # This verifies the chosen TCP point; suction footprint/contact checks remain
    # separate and must not be replaced with this center-only test.
    if np.any(np.abs(G[:2,3])>=min(dims)/2):raise ValueError('Suction target lies outside the observed top face')
    return {'cube_frame':'top_face_center_z_up','tcp_in_cube':G.tolist(),
            'offset_in_cube_mm':G[:3,3].tolist(),'observed_top_dimensions_mm':dims.tolist()}

def place_tcp(cube_top_goal_world,cube_goal_yaw_deg,grasp):
    C=cube_top_frame(cube_top_goal_world,cube_goal_yaw_deg)
    G=np.asarray(grasp['tcp_in_cube'],dtype=float)
    if G.shape!=(4,4) or not np.all(np.isfinite(G)):raise ValueError('Measured grasp transform required')
    result=matrix_pose(C@G)
    return {k:(0.0 if abs(v)<1e-10 else v) for k,v in result.items()}
