"""Aligned RGB-D localization. All outputs are millimetres in Viam world.
No estimated lens intrinsics or hard-coded marker dimensions are used.
"""
import math
import copy
import cv2
import numpy as np
from .find_cube_ring import find_objects
from .transforms import pose_matrix

def decode_depth(blob):
    if blob[:8]!=b'DEPTHMAP':raise ValueError('Expected Viam DEPTHMAP')
    w=int.from_bytes(blob[8:16],'big');h=int.from_bytes(blob[16:24],'big')
    if not 0<w<=4096 or not 0<h<=4096 or len(blob)!=24+w*h*2:raise ValueError('Invalid depth dimensions')
    return np.frombuffer(blob[24:],dtype='>u2').reshape(h,w).astype(float)

def intrinsics(props,shape):
    intr=props.get('intrinsicParameters',{})
    keys=['widthPx','heightPx','focalXPx','focalYPx','centerXPx','centerYPx']
    if any(k not in intr for k in keys):raise ValueError('Measured camera intrinsics are required')
    if any(not math.isfinite(intr[k]) for k in keys):raise ValueError('Invalid intrinsics')
    h,w=shape
    if (intr['heightPx'],intr['widthPx'])!=(h,w):raise ValueError('Image does not match calibrated intrinsics')
    if intr['focalXPx']<=0 or intr['focalYPx']<=0:raise ValueError('Invalid focal length')
    if any(abs(x)>1e-8 for x in props.get('distortionParameters',{}).get('parameters',[])):
        raise ValueError('Nonzero distortion requires calibrated undistortion before targeting')
    return intr

def world_cloud(depth,intr,pose):
    v,u=np.indices(depth.shape)
    xyz=np.stack(((u-intr['centerXPx'])*depth/intr['focalXPx'],(v-intr['centerYPx'])*depth/intr['focalYPx'],depth),axis=-1)
    matrix=np.array(pose_matrix(pose))
    return xyz@matrix[:3,:3].T+matrix[:3,3]

def fit_horizontal_plane(points, tolerance=4):
    if len(points)<150:raise ValueError('Too few valid depth samples')
    pts=points[::max(1,len(points)//5000)]
    rng=np.random.default_rng(7);best=None
    for _ in range(100):
        p=pts[rng.choice(len(pts),3,replace=False)]
        n=np.cross(p[1]-p[0],p[2]-p[0]);norm=np.linalg.norm(n)
        if norm<1e-8:continue
        n=n/norm
        if n[2]<0:n=-n
        if n[2]<math.cos(math.radians(12)):continue
        d=float(n@p[0]);mask=np.abs(pts@n-d)<tolerance
        if best is None or mask.sum()>best[0]:best=(int(mask.sum()),n,d)
    if best is None or best[0]<150:raise ValueError('No horizontal surface: inspect camera extrinsics/tilt')
    mask=np.abs(points@best[1]-best[2])<tolerance
    inliers=points[mask];center=np.mean(inliers,axis=0)
    _,_,vt=np.linalg.svd(inliers[::max(1,len(inliers)//5000)]-center,full_matrices=False)
    n=vt[-1];n=n if n[2]>0 else -n
    if n[2]<math.cos(math.radians(12)):raise ValueError('Surface tilt exceeds 12 degrees')
    d=float(n@center)
    return n,d,inliers

def ray_plane(uv,intr,pose,n,d):
    T=np.array(pose_matrix(pose));origin=T[:3,3]
    ray=T[:3,:3]@np.array([(uv[0]-intr['centerXPx'])/intr['focalXPx'],(uv[1]-intr['centerYPx'])/intr['focalYPx'],1])
    den=float(n@ray)
    if abs(den)<.1:raise ValueError('Camera ray too oblique')
    t=(d-n@origin)/den
    if not 100<t<2000:raise ValueError('Target depth outside supported range')
    return origin+t*ray

def native(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,dict):return {k:native(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [native(v) for v in value]
    return value

def downward_pose(xyz,yaw_deg):
    # Parallel (opposite direction is equivalent for a square face), choosing
    # the branch near the successful taught tool rotation rather than a 180 flip.
    return dict(zip(('x','y','z'),map(float,xyz)),o_x=0.,o_y=0.,o_z=-1.,theta=float((-yaw_deg+180)%360-180))

def fit_top_rectangle(xy):
    """Trim noise in each candidate edge frame, never along fixed world axes.
    World-axis trimming otherwise biases a rotated cube toward zero yaw.
    """
    # Trim sparse depth-edge outliers before fitting the beveled top face.
    # Full-plane image-edge and observed-corner checks remain independent.
    points=xy[::max(1,len(xy)//5000)];best=None
    for yaw in np.arange(-45,45,.25):
        a=math.radians(yaw);R=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
        projected=points@R
        low=np.percentile(projected,.5,axis=0);high=np.percentile(projected,99.5,axis=0)
        dims=high-low;area=float(np.prod(dims))
        if best is None or area<best[0]:best=(area,((high+low)/2)@R.T,dims,float(yaw))
    return best[1],sorted(best[2]),best[3]

def validate_face_corners(xy,yaw,n,d,intr,pose,shape):
    a=math.radians(yaw);R=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    projected=xy@R;lo=np.percentile(projected,.5,axis=0);hi=np.percentile(projected,99.5,axis=0)
    corners=np.array([[lo[0],lo[1]],[lo[0],hi[1]],[hi[0],hi[1]],[hi[0],lo[1]]])@R.T
    inv=np.linalg.inv(np.array(pose_matrix(pose)));h,w=shape
    for corner in corners:
        z=(d-n[0]*corner[0]-n[1]*corner[1])/n[2]
        p=inv@np.array([*corner,z,1]);u=intr['focalXPx']*p[0]/p[2]+intr['centerXPx'];v=intr['focalYPx']*p[1]/p[2]+intr['centerYPx']
        if not(8<u<w-8 and 8<v<h-8):raise ValueError('cube top surface is clipped by image edge (projected corner)')
        if np.min(np.linalg.norm(xy-corner,axis=1))>18:raise ValueError('cube top surface is clipped by image edge or has an unobserved corner')


def consensus_observation(observations):
    """Four of five frames must agree, including the two newest frames.
    5 mm / 3 degree tolerances reject isolated depth noise. The ring is optional,
    but must then be absent or present consistently.
    """
    if len(observations)!=5:raise ValueError('Five observations required')
    stamps=[o['capture_time'] for o in observations]
    if len(set(stamps))!=5:raise ValueError('Camera returned repeated frames')
    qs=np.array([o['observation_joints_deg'] for o in observations])
    if np.max(np.ptp(qs,axis=0))>.05:raise ValueError('Arm moved while confirming detections')
    has_ring={'ring' in o for o in observations}
    if len(has_ring)!=1:raise ValueError('Ring detected in only some frames')
    has_ring=has_ring.pop()
    cubes=np.array([o['cube']['top_center_world_mm'] for o in observations])
    rings=np.array([o['ring']['top_center_world_mm'] for o in observations]) if has_ring else np.zeros_like(cubes)
    angles=np.array([o['cube']['world_yaw_deg'] for o in observations])
    differences=(angles[:,None]-angles[None,:]+45)%90-45
    reference=angles[np.argmin(np.sum(np.abs(differences),axis=1))]
    unwrapped=reference+(angles-reference+45)%90-45
    angle=float(np.median(unwrapped))
    dc=np.linalg.norm(cubes-np.median(cubes,axis=0),axis=1)
    dr=np.linalg.norm(rings-np.median(rings,axis=0),axis=1)
    da=np.abs((angles-angle+45)%90-45)
    good=(dc<=5)&(dr<=5)&(da<=3)
    if int(good.sum())<4 or not all(good[-2:]):
        raise ValueError(f'Detections unstable: agreeing frames {int(good.sum())}/5, center deviation {max(dc.max(),dr.max()):.1f} mm, edge deviation {da.max():.1f} deg')
    result=copy.deepcopy(observations[-1])
    result['cube']['top_center_world_mm']=np.median(cubes[good],axis=0).tolist()
    result['cube']['world_yaw_deg']=float((np.median(unwrapped[good])+45)%90-45)
    result['cube']['observed_height_mm']=float(np.median([o['cube']['observed_height_mm'] for o,g in zip(observations,good) if g]))
    if has_ring:
        result['ring']['top_center_world_mm']=np.median(rings[good],axis=0).tolist()
        result['ring']['height_mm']=float(np.median([o['ring']['height_mm'] for o,g in zip(observations,good) if g]))
        result['ring']['bounding_diameter_mm']=max(o['ring']['bounding_diameter_mm'] for o,g in zip(observations,good) if g)
    result['detection_quality']={'frames':5,'agreeing_frames':int(good.sum()),'max_inlier_center_deviation_mm':float(max(dc[good].max(),dr[good].max())),'max_inlier_edge_deviation_deg':float(da[good].max())}
    return result

def analyze(bgr,depth,props,camera_pose,attrs,require_ring=True):
    """Locate the white cube (top face) and, when visible, the black ring.

    attrs supplies cube_size_mm and table_z_mm. The ring is required for ring goals;
    for box goals it is optional and only used as a collision obstacle.
    """
    if bgr.shape[:2]!=depth.shape:raise ValueError('Color/depth sizes differ')
    intr=intrinsics(props,depth.shape)
    cloud=world_cloud(depth,intr,camera_pose)
    valid=(depth>150)&(depth<1800)&np.isfinite(depth)
    cubes,rings=find_objects(bgr)
    result={'cube_count':len(cubes),'ring_count':len(rings),'issues':[],
            'camera_pose_world':camera_pose,'camera_intrinsics':intr,'nominal_cube_size_mm':attrs.get('cube_size_mm',200)}
    if len(cubes)!=1:result['issues'].append('Need exactly one full cube in view')
    if len(rings)>1 or (require_ring and len(rings)!=1):result['issues'].append('Need exactly one full ring in view')
    if result['issues']:return native(result)
    h,w=depth.shape;cube=cubes[0];ring=rings[0] if rings else None
    for label,obj in [('cube',cube),('ring',ring)]:
        if obj is None:continue
        x,y,bw,bh=obj['bbox']
        if min(x,y)<8 or x+bw>w-8 or y+bh>h-8:
            result['issues'].append(label+' is clipped by image edge')
    if result['issues']:return native(result)
    cube_mask=np.zeros(depth.shape,np.uint8);cv2.fillConvexPoly(cube_mask,np.asarray(cube['box'],np.int32),1)
    ring_outer=np.zeros(depth.shape,np.uint8)
    if ring is not None:cv2.ellipse(ring_outer,ring['ellipse'],1,-1)
    hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
    # Table samples: white low-saturation pixels outside all object detections.
    excluded=cv2.dilate(cube_mask|ring_outer,np.ones((35,35),np.uint8))
    table_mask=valid&(excluded==0)&(hsv[:,:,1]<55)&(hsv[:,:,2]>100)
    tn,td,table_points=fit_horizontal_plane(cloud[table_mask])
    table_z=float(np.median(table_points[:,2]));result['observed_table_z_mm']=table_z
    expected=attrs.get('table_z_mm',-23)
    result['table_reference_error_mm']=table_z-expected
    if abs(table_z-expected)>15:result['issues'].append('Depth/world tabletop differs from configured table by more than 15 mm; camera calibration needs verification')
    # Refine the cube using its horizontal TOP surface in 3D, not the 2D rectangle
    # that can include a visible side face. This also gives the edge yaw in world.
    nominal=float(attrs.get('cube_size_mm',200))
    top_mask=valid&(cube_mask>0)&(cloud[:,:,2]>table_z+nominal*.65)&(cloud[:,:,2]<table_z+nominal*1.3)
    cn,cd,top_points=fit_horizontal_plane(cloud[top_mask])
    # The detector encloses printed corner marks, not the whole top face.
    # Grow onto the connected coplanar surface before estimating its edges.
    coplanar=valid&(np.abs(cloud@cn-cd)<3)&(hsv[:,:,1]<70)
    count,labels=cv2.connectedComponents(coplanar.astype(np.uint8))
    candidates=[(int(np.sum((labels==i)&top_mask)),i) for i in range(1,count)]
    if not candidates or max(candidates)[0]<150:raise ValueError('Cube top surface is not connected')
    face=labels==max(candidates)[1]
    rows,cols=np.nonzero(face)
    if min(rows.min(),cols.min())<8 or rows.max()>=h-8 or cols.max()>=w-8:
        result['issues'].append('cube top surface is clipped by image edge')
        return native(result)
    top_points=cloud[face]
    xy=top_points[:,:2]
    center_xy,dims,yaw=fit_top_rectangle(xy)
    center=np.array([*center_xy,0.]);center[2]=(cd-cn[0]*center[0]-cn[1]*center[1])/cn[2]
    try:validate_face_corners(xy,yaw,cn,cd,intr,camera_pose,(h,w))
    except ValueError as e:
        result['issues'].append(str(e));return native(result)
    observed_height=float(center[2]-table_z)
    if not all(nominal*.75<d<nominal*1.2 for d in dims):result['issues'].append('Detected cube top dimensions do not match nominal ore size')
    if abs(observed_height-nominal)>20:result['issues'].append('Cube height does not match nominal ore size')
    result['cube']={'top_center_world_mm':center,'world_yaw_deg':yaw,'top_dimensions_mm':dims,'observed_height_mm':observed_height,'top_normal_world':cn,'pixel_hint':cube['center']}
    if ring is None:return native(result)
    # Ring: sample its dark annulus, never the tabletop visible through its hole.
    inner=np.zeros(depth.shape,np.uint8)
    (ec,axes,angle)=ring['ellipse'];cv2.ellipse(inner,(ec,(axes[0]*.62,axes[1]*.62),angle),1,-1)
    rim=valid&(ring_outer>0)&(inner==0)&(hsv[:,:,2]<100)
    rn,rd,rim_points=fit_horizontal_plane(cloud[rim],tolerance=5)
    ringcenter=ray_plane(ring['center'],intr,camera_pose,rn,rd)
    ringheight=float(ringcenter[2]-table_z)
    if not 3<ringheight<100:result['issues'].append('Ring height could not be resolved from annulus depth')
    x,y,bw,bh=ring['bbox']
    corners=cv2.boxPoints(cv2.minAreaRect(np.array([[[x,y]],[[x+bw,y]],[[x+bw,y+bh]],[[x,y+bh]]],np.float32)))
    ring_bounds=np.array([ray_plane(p,intr,camera_pose,rn,rd) for p in corners])
    diam=float(max(np.ptp(ring_bounds[:,0]),np.ptp(ring_bounds[:,1])))
    result['ring']={'top_center_world_mm':ringcenter,'height_mm':ringheight,'bounding_diameter_mm':diam,'pixel_hint':ring['center']}
    return native(result)
