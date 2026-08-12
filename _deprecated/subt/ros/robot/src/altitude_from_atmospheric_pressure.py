#!/usr/bin/python

import copy
import math
import sys

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import FluidPressure

class AltitudeFromAtmosphericPressure:
    ''' Converts atmospheric presure to altitude.

    The resulting altitude is represented as a z coordinate in an otherwise
    empty Pose message. It is relative to the starting altitude.
    '''
    def __init__(self, frame_id):
        self.frame_id = frame_id
        self.altitude_subscriber = rospy.Subscriber('/atmospheric_pressure', FluidPressure, self.pressureCallback)
        self.pose_publisher = rospy.Publisher("/pose", PoseWithCovarianceStamped, queue_size=10)

    def pressureCallback(self, pressure):
        pose = PoseWithCovarianceStamped()
        pose.header = copy.deepcopy(pressure.header)
        pose.header.frame_id = self.frame_id
        pose.pose.pose.position.z = self.__altitude_from_pressure(pressure.fluid_pressure)
        pose.pose.pose.orientation.w = 1
        # Initialize covariance with dummy values.
        for i in range(6):
            pose.pose.covariance[i * 6 + i] = 1.0
        PRESSURE_SENSOR_COVARIANCE = 1e-15 ** 2  # No noise modelled with "very little noise."
        pose.pose.covariance[2 * 6 + 2] = PRESSURE_SENSOR_COVARIANCE
        self.pose_publisher.publish(pose)

    def __altitude_from_pressure(self, p):
        # Model and parameters taken from Wikipedia:
        # https://en.wikipedia.org/wiki/Barometric_formula
        # I have not yet found corresponding atmospheric model and constants
        # in Ignition Gazebo.

        # Reference pressure. [Pa]
        Pb = 101325.00
        # Reference temperature. [K]
        Tb = 288.15
        # Gravitational acceleration. [m/s^2]
        g0 = 9.80665
        # Universal gas constant. [J/(mol*K)]
        R = 8.3144598
        # Molar mass of air. [kg/mol]
        M = 0.0289644
        # Height of reference level. [m]
        hb = 0

        # The pressure model is:
        #   p = Pb * exp(-g0 * M * (h - hb) / (R * Tb))
        # Therefore:
        h = (math.log(p) - math.log(Pb)) * (R * Tb) / (-g0 * M) + hb

        return h


if __name__ == "__main__":
    rospy.init_node("altitude_from_atmospheric_pressure", log_level=rospy.DEBUG)
    overrider = AltitudeFromAtmosphericPressure(rospy.myargv(sys.argv)[1])
    rospy.spin()
