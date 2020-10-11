#!/usr/bin/python

import copy
import rospy
from sensor_msgs.msg import Imu


class ImuCovarianceOverride:
    #class that listens all topics needed for controlling drone height
    def __init__(self, orientation_stdev, angular_stdev, acceleration_stdev):
        self.orientation_stdev = orientation_stdev
        self.angular_stdev = angular_stdev
        self.acceleration_stdev = acceleration_stdev

        self.subscriber = rospy.Subscriber("/imu/in", Imu, self.imu_callback, queue_size=15)
        self.publisher = rospy.Publisher("/imu/out", Imu, queue_size=50)


    def imu_callback(self, imu_orig):
        imu_fixed = copy.deepcopy(imu_orig)
        imu_fixed.orientation_covariance = [0.0] * 9
        imu_fixed.angular_velocity_covariance = [0.0] * 9
        imu_fixed.linear_acceleration_covariance = [0.0] * 9
        for i in range(3):
            for j in range(3):
                idx = 3 * i + j

                if i == j:
                    imu_fixed.orientation_covariance[idx] = self.orientation_stdev**2
                    imu_fixed.angular_velocity_covariance[idx] = self.angular_stdev**2
                    imu_fixed.linear_acceleration_covariance[idx] = self.acceleration_stdev**2
                else:
                    imu_fixed.orientation_covariance[idx] = 0
                    imu_fixed.angular_velocity_covariance[idx] = 0
                    imu_fixed.linear_acceleration_covariance[idx] = 0

        self.publisher.publish(imu_fixed)

if __name__ == "__main__":
    import argparse
    import sys
    parser = argparse.ArgumentParser()
    parser.add_argument('--orientation-stdev', type=float, default=1e-3,
                        help='Standard deviation of the compass noise.')
    parser.add_argument('--angular-stdev', type=float, default=1e-3,
                        help='Standard deviation of the gyroscope velocity  noise.')
    parser.add_argument('--acceleration-stdev', type=float, default=1e-3,
                        help='Standard deviation of the accelerometer noise.')
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])

    rospy.init_node("imu_covariance_override", log_level=rospy.DEBUG)
    overrider = ImuCovarianceOverride(
            args.orientation_stdev, args.angular_stdev, args.acceleration_stdev)
    rospy.spin()
