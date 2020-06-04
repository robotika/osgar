"""
  Quaternion - defined as (x, y, z, w) to be compatible with ROS and RealSense

  ROS:
    http://wiki.ros.org/tf2/Tutorials/Quaternions

ROS uses quaternions to track and apply rotations. A quaternion has 4
components (x,y,z,w). That's right, 'w' is last (but beware: some libraries
like Eigen put w as the first number!). The commonly-used unit quaternion that
yields no rotation about the x/y/z axes is (0,0,0,1)

https://www.andre-gaschler.com/rotationconverter/
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
    qvector = vector + [0]
    con = conjugate(quaternion)
    part1 = multiply(quaternion, qvector)
    return multiply(part1, con)[:-1]

def euler_zyx(quaternion):
    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    # assumption that quaternion is normalized
    x0, y0, z0, w0 = quaternion
    sqr_size = x0*x0 + y0*y0 + z0*z0 + w0*w0
    if abs(sqr_size - 1.0) > 0.00001:
        # the assumption is broken, so normalize it
        k = math.sqrt(sqr_size)
        x0, y0, z0, w0 = x0/k, y0/k, z0/k, w0/k
    ax =  math.atan2(2*(w0*x0+y0*z0), 1-2*(x0*x0+y0*y0))
    ay =  math.asin(2*(w0*y0-z0*x0))
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

# vim: expandtab sw=4 ts=4
