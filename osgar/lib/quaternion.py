
import math

def multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return [-x1*x0 - y1*y0 - z1*z0 + w1*w0,
             x1*w0 + y1*z0 - z1*y0 + w1*x0,
            -x1*z0 + y1*w0 + z1*x0 + w1*y0,
             x1*y0 - y1*x0 + z1*w0 + w1*z0]

def conjugate(quaternion):
    w, x, y, z = quaternion
    return [w, -x, -y, -z]

def identity():
    return [1, 0, 0, 0]

def rotate_vector(vector, quaternion):
    qvector = [0] + vector
    con = conjugate(quaternion)
    part1 = multiply(quaternion, qvector)
    return multiply(part1, con)[1:]

def euler_zyx(quaternion):
    w0, x0, y0, z0 = quaternion
    ax =  math.atan2(2*(w0*x0+y0*z0), 1-2*(x0*x0+y0*y0))
    ay =  math.asin(2*(w0*y0-z0*x0))
    az =  math.atan2(2*(w0*z0+x0*y0), 1-2*(y0*y0+z0*z0))
    return [az, ay, ax]

def heading(quaternion):
    w0, x0, y0, z0 = quaternion
    az =  math.atan2(2*(w0*z0+x0*y0), 1-2*(y0*y0+z0*z0))
    return az

# vim: expandtab sw=4 ts=4
