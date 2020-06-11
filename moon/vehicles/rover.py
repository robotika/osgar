"""
  Moon Rover Driver
"""

# source: (limited access)
#   https://gitlab.com/scheducation/srcp2-competitors/-/wikis/Documentation/API/Simulation_API
# Motor Drive Command Topics
#  /name/fl_wheel_controller/command
#  /name/fr_wheel_controller/command
#  /name/bl_wheel_controller/command
#  /name/br_wheel_controller/command

# Steering Arm Control Topics
#  /name/fr_steering_arm_controller/command
#  /name/fl_steering_arm_controller/command
#  /name/bl_steering_arm_controller/command
#  /name/br_steering_arm_controller/command

# Info
# /name/joint_states  sensor_msgs/JointStates
# /name/skid_cmd_vel  geometry_msgs/Twist
# /name/get_true_pose

# Sensors
# /name/laser/scan  sensor_msgs/LaserScan
# /name/camera/<side>/image_raw  sensor_msgs/Image
# /name/imu  sensor_msgs/Imu

# /name/joint_states  sensor_msgs/JointStates
# This is a message that holds data to describe the state of a set of torque controlled joints.
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.

# sensor_joint
# bl_arm_joint
# bl_steering_arm_joint
# bl_wheel_joint
# br_arm_joint
# br_steering_arm_joint
# br_wheel_joint
# fl_arm_joint
# fl_steering_arm_joint
# fl_wheel_joint
# fr_arm_joint
# fr_steering_arm_joint
# fr_wheel_joint

# Sensor Joint Controller
# /name/sensor_controller/command


import math
from datetime import timedelta

from osgar.lib.mathex import normalizeAnglePIPI
from osgar.node import Node


WHEEL_RADIUS = 0.275  # meters
WHEEL_SEPARATION_WIDTH = 1.87325  # meters
WHEEL_SEPARATION_LENGTH = 1.5748  # meters

WHEEL_NAMES = ['fl', 'fr', 'bl', 'br']

CRAB_ROLL_ANGLE = 0.78

class Rover(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('cmd', 'pose2d')

        # general driving parameters
        # radius: radius of circle to drive around, "inf" if going straight; 0 if turning round in place
        # positive if turning left, negative if turning right
        # camera_angle: direction of camera vs tangent to the circle; ie 0 if looking and going straight or doing regular turn; positive it looking sideways to the left; negative if looking sideways to the right
        # when turning in place, positive speed turns counterclockwise, negative clockwise
        self.drive_radius = float("inf") # in degrees * 100
        self.drive_camera_angle = 0  # in degrees * 100
        self.drive_speed = 0 # 0 and non-zero only for now


        self.desired_linear_speed = 0.0  # m/s
        self.desired_angular_speed = 0.0

        self.joint_name = None  # updated via Node.update()
        self.debug_arr = []
        self.verbose = False
        self.prev_position = None
        self.pose2d = 0, 0, 0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.yaw_offset = None
        self.in_driving_recovery = False

    def on_driving_recovery(self, data):
        self.in_driving_recovery = data

    def on_desired_speed(self, data):
        # self.desired_linear_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)
        linear, angular = data # legacy: mutually exclusive, either linear goes straigth or angular turns in place; both 0 is stop
        if linear == 0 and angular == 0:
            self.drive_radius = self.drive_speed = 0
        elif angular != 0:
            self.drive_radius = 0 # turn in place
            self.drive_speed =  math.copysign(10, angular)
        else: # linear is non-zero
            self.drive_radius = float("inf") # going straight
            self.drive_speed = linear
        self.drive_camera_angle = 0 # only 0, positive (looking left 90 degrees when going forward) and negative (right) are supported


    def on_desired_movement(self, data):
        # rover will go forward in a circle given:
        # circle radius (m) (use float("inf") to go straight)
        # angle between the center of the circle and the direction of the camera (angle in 100*degrees, positive if center to the left of the camera); NOTE: currently only the sign matters and will result in looking left and right 90 degrees respectively
        # linear speed (1000*m/s) NOTE: speed only considers the sign for going forward, backward or stop
        self.drive_radius, self.drive_camera_angle, self.drive_speed = data

    def on_rot(self, data):
        rot = data
        (temp_yaw, self.pitch, self.roll) = [normalizeAnglePIPI(math.radians(x/100)) for x in rot]

        if self.yaw_offset is None:
            self.yaw_offset = -temp_yaw
        self.yaw = temp_yaw + self.yaw_offset
        #print ("yaw: %f, pitch: %f, roll: %f" % (self.yaw, self.pitch, self.roll))

    def on_joint_position(self, data):
        assert self.joint_name is not None
        if self.prev_position is None:
            self.prev_position = data

        diff = [b - a for a, b in zip(self.prev_position, data)]

        assert b'bl_wheel_joint' in self.joint_name, self.joint_name
        if self.desired_linear_speed >= 0:
            name = b'bl_wheel_joint'
            name2 = b'br_wheel_joint'
        else:
            name = b'fl_wheel_joint'
            name2 = b'fr_wheel_joint'
        left = WHEEL_RADIUS * diff[self.joint_name.index(name)]
        right = WHEEL_RADIUS * diff[self.joint_name.index(name2)]
        dist = (left + right)/2
        angle = (right - left)/WHEEL_SEPARATION_WIDTH
        x, y, heading = self.pose2d
        x += math.cos(heading) * dist
        y += math.sin(heading) * dist
        heading += angle
        self.bus.publish('pose2d', [round(x * 1000),
                                    round(y * 1000),
                                    round(math.degrees(heading) * 100)])
        self.prev_position = data
        self.pose2d = x, y, heading

    def on_joint_velocity(self, data):
        assert self.joint_name is not None
        speed = []
        for wheel in WHEEL_NAMES:  # cycle through fl, fr, bl, br
            speed.append(WHEEL_RADIUS * data[self.joint_name.index(bytes(wheel, 'ascii') + b'_wheel_joint')])
        if self.verbose:
            self.debug_arr.append([self.time.total_seconds(),] + speed)

    def on_joint_effort(self, data):
        assert self.joint_name is not None

        # TODO cycle through fl, fr, bl, br
        effort =  data[self.joint_name.index(b'fl_wheel_joint')]

        steering = [0.0,] * 4

        # turning in place if radius is 0 but speed is non-zero
        if self.drive_radius == 0:
            e = 40
            if self.drive_speed == 0:
                effort = [0,] * 4
            elif self.drive_speed > 0:
                # turn left
                effort = [-e, e, -e, e]
                steering = [-CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE,-CRAB_ROLL_ANGLE]
            else: # drive speed < 0
                # turn right
                effort = [e, -e, e, -e]
                steering = [-CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE,-CRAB_ROLL_ANGLE]

        else:

            # if pitch too steep, turn diagonally a try to climb this way being able to handle larger pitch
            if self.pitch > 0.3:
                # TODO: try to maintain wheel orientation up (wheels turning straight up even if body turns)
                if self.roll > 0:
                    #steering = [3*-self.roll,3*-self.roll,0.0, 0.0]

                    steering = [-CRAB_ROLL_ANGLE,-CRAB_ROLL_ANGLE,-CRAB_ROLL_ANGLE,-CRAB_ROLL_ANGLE]
                else:
                    #steering = [3*-self.roll,3*-self.roll,0.0, 0.0]
                    steering = [CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE]
                e = e2 = 120
                effort = [e, e, e, e]

            else:
                # TODO: if large change of 'steering' values, allow time to apply before turning on 'effort'
                camera_angle = math.pi * (self.drive_camera_angle / 100.0) / 180.0
                fl = fr = rl = rr = 0.0

                e = 80 if self.drive_speed > 0 else -80
                effort = [e, e, e, e]

                if not math.isinf(self.drive_radius):
                    sign = 1 if self.drive_radius > 0 else -1
                    signed_width = -math.copysign(WHEEL_SEPARATION_WIDTH/2.0, self.drive_radius)
                    fl = sign * WHEEL_SEPARATION_LENGTH / (abs(self.drive_radius) + signed_width) # + if outer
                    fr = sign * WHEEL_SEPARATION_LENGTH / (abs(self.drive_radius) - signed_width)
                    rl = sign * -WHEEL_SEPARATION_LENGTH / (abs(self.drive_radius) + signed_width)
                    rr = sign * -WHEEL_SEPARATION_LENGTH / (abs(self.drive_radius) - signed_width)

                    if camera_angle > 0:
                        if self.drive_radius > 0:
                            temp = rr
                            rr = -math.pi/2 + fr
                            fr = -math.pi/2 + fl
                            fl = math.pi/2 + rl
                            rl = math.pi/2 + temp
                            effort = [-e, e, -e, e]
                        else:
                            temp = rr
                            rr = math.pi/2 + fr
                            fr = math.pi/2 + fl
                            fl = -math.pi/2 + rl
                            rl = -math.pi/2 + temp
                            effort = [e, -e, e, -e]
                    elif camera_angle < 0:
                        if self.drive_radius > 0:
                            temp = rr
                            rr = math.pi/2 + rl
                            rl = -math.pi/2 + fl
                            fl = -math.pi/2 + fr
                            fr = math.pi/2 + temp
                            effort = [-e, e, -e, e]
                        else:
                            temp = rr
                            rr = -math.pi/2 + rl
                            rl = math.pi/2 + fl
                            fl = math.pi/2 + fr
                            fr = -math.pi/2 + temp
                            effort = [e, -e, e, -e]


                if self.drive_camera_angle == 0:
                    # during normal driving, steer against slope proportionately to the steepness of the slope
                    fl -= self.roll
                    fr -= self.roll
                    rl -= self.roll
                    rr -= self.roll

                steering = [fl, fr, rl, rr]

                # stay put while joint angles are catching up
                if self.drive_camera_angle != 0 and self.prev_position is not None and not self.in_driving_recovery:
                    if (
                            abs(self.prev_position[self.joint_name.index(b'bl_steering_arm_joint')] - rl) > 0.2 or
                            abs(self.prev_position[self.joint_name.index(b'br_steering_arm_joint')] - rr) > 0.2 or
                            abs(self.prev_position[self.joint_name.index(b'fl_steering_arm_joint')] - fl) > 0.2 or
                            abs(self.prev_position[self.joint_name.index(b'fr_steering_arm_joint')] - fr) > 0.2
                    ):
                        effort = [0.0,]*4

        cmd = b'cmd_rover %f %f %f %f %f %f %f %f' % tuple(steering + effort)
        self.bus.publish('cmd', cmd)

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))

        return channel


    def draw(self):
        # for debugging
        import matplotlib.pyplot as plt
        arr = self.debug_arr
        t = [a[0] for a in arr]
        values = [a[1:] for a in arr]

        line = plt.plot(t, values, '-', linewidth=2)

        plt.xlabel('time (s)')
        plt.legend(WHEEL_NAMES)
        plt.show()

# vim: expandtab sw=4 ts=4
