#!/usr/bin/python

import math
import sys

from geometry_msgs.msg import Twist
import rospy
import tf

class K2Controller:
    """
      4WD controller for robot Kloubak K2
    """
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.joint_listener = tf.TransformListener()
        self.input_subscriber = rospy.Subscriber('/cmd_vel/input', Twist, self.inputCallback)
        self.output_front_publisher = rospy.Publisher("/cmd_vel/output/front", Twist, queue_size=10)
        self.output_rear_publisher = rospy.Publisher("/cmd_vel/output/rear", Twist, queue_size=10)

        # K2's control needs to be frequent enough to account for the current
        # state of the middle joint angle. However, high level control commands,
        # such as "go straight" or "rotate on the spot" can come only once in
        # a blue moon. Therefore we establish a high frequency control loop
        # which takes last requested command into account.
        self.desired = Twist()
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control)


    def inputCallback(self, input_cmd):
        self.desired = input_cmd


    def control(self, _event):
        # The idea of the control algorithm is as follows:
        #
        # The angle of the central joint defines an arc along which the robot
        # is traveling. Forward velocity along this arc causes rotation of the
        # robot. Given desired rotation velocity and forward velocity, we can
        # calculate backwards what the central joint angle should be.
        #
        # Once we know the desired joint angle and current actual joint angle,
        # we can rotate front and rear halves of the robot in opposite
        # directions to minimize the difference.
        #
        # Rotation of one half of the robot, however, causes pull or push of the
        # other half, hence we need to make it move forward faster or slower
        # accordingly.

        # Distance between axes and the middle hinge.
        # For simplicity, we are neglecting that the front distance (35 cm)
        # differs from rear distance (37.9 cm).
        L = 0.3645
        # The maximum possible angle of the middle joint.
        MAX_JOINT_ANGLE = math.radians(70)

        try:
            (_trans, joint_rot) = self.joint_listener.lookupTransform(self.robot_name + '/chassis_back', self.robot_name + '/chassis_front', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # Not ready yet.
            return

        current_joint_angle = tf.transformations.euler_from_quaternion(joint_rot)[2]

        front_cmd = Twist()
        rear_cmd = Twist()

        # K2 cannot rotate with a forward/backward velocity.
        if self.desired.linear.x == 0:
            self.output_front_publisher.publish(front_cmd)
            self.output_rear_publisher.publish(rear_cmd)
            return

        # Initial setting that will be corrected based on joint angle and
        # desired rotations of each half of the robot.
        front_cmd.linear.x = self.desired.linear.x
        rear_cmd.linear.x = self.desired.linear.x

        # Initial rotation needed to keep the robot on the current arc without
        # slipping.
        front_cmd.angular.z = rear_cmd.angular.z = (
                self.desired.linear.x * math.tan(current_joint_angle / 2) / L)

        desired_joint_angle = max(
                -MAX_JOINT_ANGLE,
                min(MAX_JOINT_ANGLE,
                    2 * math.atan2(self.desired.angular.z * L,
                                   self.desired.linear.x)))
        #print('current', [math.degrees(alpha) for alpha in euler_zyx(joint_rot)], 'desired', math.degrees(desired_joint_angle), 'because of', math.degrees(self.desired.angular.z))

        P = 5.0  # P control.
        rot = desired_joint_angle - current_joint_angle

        # To work together, the halfs of the robot need to rotate in opposite
        # directions.
        front_cmd.angular.z += P * rot
        rear_cmd.angular.z += -P * rot

        # Rotating one half of the robot pulls/pushes the other one
        # forward/backward. To reduce wheel slipping and keep control, we need
        # to account  for this effect.
        front_cmd.linear.x += math.sin(current_joint_angle) * rear_cmd.angular.z * L
        rear_cmd.linear.x += math.sin(current_joint_angle) * front_cmd.angular.z * L

        self.output_front_publisher.publish(front_cmd)
        self.output_rear_publisher.publish(rear_cmd)


if __name__ == "__main__":
    rospy.init_node("k2_contoller", log_level=rospy.DEBUG)
    overrider = K2Controller(rospy.myargv(sys.argv)[1])
    rospy.spin()
