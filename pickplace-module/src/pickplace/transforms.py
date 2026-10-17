import math
def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rotation(axis, angle):
    n = math.sqrt(sum(x*x for x in axis))
    x, y, z = [v/n for v in axis]
    c, s, v = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    r = identity()
    r[:3] = [[x*x*v+c, x*y*v-z*s, x*z*v+y*s, 0],
             [y*x*v+z*s, y*y*v+c, y*z*v-x*s, 0],
             [z*x*v-y*s, z*y*v+x*s, z*z*v+c, 0]]
    return r


def ov_rotation(x, y, z, theta):
    norm = math.sqrt(x*x+y*y+z*z)
    if not norm:
        x, y, z, norm = 0, 0, 1, 1
    x, y, z = x/norm, y/norm, z/norm
    lon, lat = math.atan2(y,x), math.acos(max(-1,min(1,z)))
    if 1-abs(z) <= 0.0001:
        theta += -lon if z > 0 else lon
    return multiply(multiply(rotation([0,0,1],lon),rotation([0,1,0],lat)), rotation([0,0,1],theta))


def orientation(o):
    if not o or not o.get('type'):
        return identity()
    v = o['value']
    if o['type'] == 'ov_degrees':
        return ov_rotation(v.get('x',0),v.get('y',0),v.get('z',1),math.radians(v.get('th',0)))
    if o['type'] == 'euler_angles':
        return multiply(multiply(rotation([0,0,1],v['yaw']),rotation([0,1,0],v['pitch'])),rotation([1,0,0],v['roll']))
    raise ValueError('Unsupported frame orientation: '+o['type'])


def pose_matrix(p):
    r = ov_rotation(p.get('oX',p.get('o_x',0)), p.get('oY',p.get('o_y',0)),p.get('oZ',p.get('o_z',1)),math.radians(p.get('theta',0)))
    for i,k in enumerate(('x','y','z')):
        r[i][3] = p.get(k,0)
    return r


def matrix_pose(m):
    x,y,z = [m[i][2] for i in range(3)]
    lon = math.atan2(y,x)
    # R = Rz(lon) Ry(lat) Rz(theta)
    theta = math.atan2(m[2][1],-m[2][0])
    if 1-abs(z) <= 0.0001:
        theta = math.atan2(m[1][0],m[0][0]) if z>0 else math.atan2(m[1][0],-m[0][0])
    return dict(zip(('x','y','z','o_x','o_y','o_z','theta'),
                    [m[0][3],m[1][3],m[2][3],x,y,z,math.degrees(theta)]))

