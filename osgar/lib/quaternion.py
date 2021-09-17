"""
  Quaternion - defined as (x, y, z, w) to be compatible with ROS and RealSense

  ROS:
    http://wiki.ros.org/tf2/Tutorials/Quaternions

ROS uses quaternions to track and apply rotations. A quaternion has 4
components (x,y,z,w). That's right, 'w' is last (but beware: some libraries
like Eigen put w as the first number!). The commonly-used unit quaternion that
yields no rotation about the x/y/z axes is (0,0,0,1)

https://www.andre-gaschler.com/rotationconverter/
https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
"""
import math


def multiply(quaternion1, quaternion0):
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return [ x1*w0 + y1*z0 - z1*y0 + w1*x0,
            -x1*z0 + y1*w0 + z1*x0 + w1*y0,
             x1*y0 - y1*x0 + z1*w0 + w1*z0,
            -x1*x0 - y1*y0 - z1*z0 + w1*w0]

def conjugate(quaternion):
    x, y, z, w = quaternion
    return [-x, -y, -z, w]

def identity():
    return [0, 0, 0, 1]

def rotate_vector(vector, quaternion):
    # https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    qvector = vector + [0]
    con = conjugate(quaternion)
    part1 = multiply(quaternion, qvector)
    return multiply(part1, con)[:-1]


def transform(vector, transformation):
    translation, rotation = transformation
    rotated = rotate_vector(vector, rotation)
    return [sum(v) for v in zip(rotated, translation)]


def normalize(quaternion):
    x0, y0, z0, w0 = quaternion
    sqr_size = x0*x0 + y0*y0 + z0*z0 + w0*w0
    if abs(sqr_size - 1.0) > 0.00001:
        k = math.sqrt(sqr_size)
        x0, y0, z0, w0 = x0/k, y0/k, z0/k, w0/k
    return [x0, y0, z0, w0]

def euler_to_quaternion(yaw, pitch, roll):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

def euler_zyx(quaternion):
    x0, y0, z0, w0 = normalize(quaternion)
    ax =  math.atan2(2*(w0*x0+y0*z0), 1-2*(x0*x0+y0*y0))
    ay =  math.asin(max(-1, min(1, 2*(w0*y0-z0*x0))))  # cliping the range because of posible small overflows
    az =  math.atan2(2*(w0*z0+x0*y0), 1-2*(y0*y0+z0*z0))
    return [az, ay, ax]

def heading(quaternion):
    x0, y0, z0, w0 = quaternion
    az =  math.atan2(2*(w0*z0+x0*y0), 1-2*(y0*y0+z0*z0))
    return az

def from_axis_angle(axis, angle):
    ax, ay, az = axis
    qx = ax * math.sin(angle / 2)
    qy = ay * math.sin(angle / 2)
    qz = az * math.sin(angle / 2)
    qw = math.cos(angle / 2)
    return [qx, qy, qz, qw]


def rotation_matrix(quaternion):
    qx, qy, qz, qw = quaternion
    r1 = [1 - 2*qy**2 - 2*qz**2,  2*qx*qy - 2*qz*qw,      2*qx*qz + 2*qy*qw]
    r2 = [2*qx*qy + 2*qz*qw,      1 - 2*qx**2 - 2*qz**2,  2*qy*qz - 2*qx*qw]
    r3 = [2*qx*qz - 2*qy*qw,      2*qy*qz + 2*qx*qw,      1 - 2*qx**2 - 2*qy**2]
    return [r1, r2, r3]


def from_rotation_matrix(rotation_matrix):
    # http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
    # https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
    m00, m01, m02 = rotation_matrix[0]
    m10, m11, m12 = rotation_matrix[1]
    m20, m21, m22 = rotation_matrix[2]
    qw = math.sqrt(max(0, 1 + m00 + m11 + m22)) / 2
    qx = math.sqrt(max(0, 1 + m00 - m11 - m22)) / 2
    qy = math.sqrt(max(0, 1 - m00 + m11 - m22)) / 2
    qz = math.sqrt(max(0, 1 - m00 - m11 + m22)) / 2
    qx = math.copysign(qx, m21 - m12)
    qy = math.copysign(qy, m02 - m20)
    qz = math.copysign(qz, m10 - m01)
    return [qx, qy, qz, qw]


def angle_between(quaternion0, quaternion1):
    # https://math.stackexchange.com/questions/90081/quaternion-distance
    inner_product = sum(a * b for (a, b) in zip(quaternion0, quaternion1))
    val = 2 * inner_product**2 - 1
    return math.acos(max(-1.0, min(1.0, val)))


# vim: expandtab sw=4 ts=4
