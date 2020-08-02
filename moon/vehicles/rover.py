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

from moon.moonnode import MoonNode
from moon.motorpid import MotorPID


WHEEL_RADIUS = 0.275  # meters
WHEEL_SEPARATION_WIDTH = 1.87325  # meters
WHEEL_SEPARATION_LENGTH = 1.5748  # meters

WHEEL_NAMES = ['fl', 'fr', 'bl', 'br']

CRAB_ROLL_ANGLE = 0.78

class Rover(MoonNode):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('cmd', 'odo_pose')

        # general driving parameters
        # radius: radius of circle to drive around, "inf" if going straight; 0 if turning round in place
        # positive if turning left, negative if turning right
        # camera_angle: direction of camera vs tangent to the circle; ie 0 if looking and going straight or doing regular turn; positive it looking sideways to the left; negative if looking sideways to the right
        # when turning in place, positive speed turns counterclockwise, negative clockwise
        self.drive_radius = float("inf") # in degrees * 100
        self.drive_camera_angle = 0  # in degrees * 100
        self.drive_speed = 0 # -1000 to 1000 (0.. stop, 1000 maximum feasible speed given the type of motion)

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
        self.steering_wait_start = None

        self.motor_pid = [MotorPID(p=40.0) for __ in WHEEL_NAMES]  # TODO tune PID params

    def on_driving_recovery(self, data):
        self.in_driving_recovery = data

    def on_desired_speed(self, data):
        # self.desired_linear_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)
        linear, angular = data # legacy: mutually exclusive, either linear goes straigth or angular turns in place; both 0 is stop
        if linear == 0 and angular == 0:
            self.drive_radius = self.drive_speed = 0
        elif angular != 0:
            self.drive_radius = 0 # turn in place
            self.drive_speed = 1000 * angular / (100 * 60) # max angular speed is 60 deg/sec, value provided in 100 multiple
        else: # linear is non-zero
            self.drive_radius = float("inf") # going straight
            self.drive_speed = linear
        self.drive_camera_angle = 0 # only 0, positive (looking left 90 degrees when going forward) and negative (right) are supported


    def on_desired_movement(self, data):
        # rover will go forward in a circle given:
        # circle radius (m) (use float("inf") to go straight)
        # angle between the center of the circle and the direction of the camera (angle in 100*degrees, positive if center to the left of the camera); NOTE: currently only the sign matters and will result in looking left and right 90 degrees respectively
        # drive_speed: linear speed in 1000* m/s
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
        # measure odometry from rear wheels
        name = b'bl_wheel_joint'
        name2 = b'br_wheel_joint'
        # name = b'fl_wheel_joint'
        # name2 = b'fr_wheel_joint'
        left = WHEEL_RADIUS * diff[self.joint_name.index(name)]
        right = WHEEL_RADIUS * diff[self.joint_name.index(name2)]
        dist = (left + right)/2
        angle = (right - left)/WHEEL_SEPARATION_WIDTH
        x, y, heading = self.pose2d
        x += math.cos(heading) * dist
        y += math.sin(heading) * dist
        heading += angle
        self.bus.publish('odo_pose', [round(x * 1000),
                                    round(y * 1000),
                                    round(math.degrees(heading) * 100)])
        self.prev_position = data
        self.pose2d = x, y, heading

    def on_joint_velocity(self, data):
        assert self.joint_name is not None
        speed = []
        for i, wheel in enumerate(WHEEL_NAMES):  # cycle through fl, fr, bl, br
            s = WHEEL_RADIUS * data[self.joint_name.index(bytes(wheel, 'ascii') + b'_wheel_joint')]
            speed.append(s)
            self.motor_pid[i].update(s)

        if self.verbose:
            self.debug_arr.append([self.time.total_seconds(),] + speed)

    def on_joint_effort(self, data):
        assert self.joint_name is not None
        steering, effort = self.get_steering_and_effort()

        ##### integrate PID start #####
        for i, e in enumerate(effort):
            self.motor_pid[i].set_desired_speed(e/40)  # TODO review values 0, 40, 60, 120
        effort = []
        for m in self.motor_pid:
            effort.append(m.get_effort())
        ###### integrate PID end ######
        cmd = b'cmd_rover %f %f %f %f %f %f %f %f' % tuple(steering + effort)
        self.bus.publish('cmd', cmd)

    def get_steering_and_effort(self):
        steering = [0.0,] * 4

        if self.drive_speed == 0:
            effort = [0,] * 4

        elif self.drive_radius == 0:
            # turning in place if radius is 0 but speed is non-zero
            e = 30

            if self.drive_speed > 0:
                # turn left
                effort = [-e, e, -e, e]
                steering = [-CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE,-CRAB_ROLL_ANGLE]
            else: # drive speed < 0
                # turn right
                effort = [e, -e, e, -e]
                steering = [-CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE,-CRAB_ROLL_ANGLE]

        else:
            # TODO: if large change of 'steering' values, allow time to apply before turning on 'effort'
            fl = fr = rl = rr = 0.0

            e = 80 * self.drive_speed / 1000.0
            effort = [e, e, e, e]

            if not math.isinf(self.drive_radius):
                sign = 1 if self.drive_radius > 0 else -1
                signed_width = -math.copysign(WHEEL_SEPARATION_WIDTH/2.0, self.drive_radius)
                fl = sign * WHEEL_SEPARATION_LENGTH / (abs(self.drive_radius) + signed_width) # + if outer
                fr = sign * WHEEL_SEPARATION_LENGTH / (abs(self.drive_radius) - signed_width)
                rl = sign * -WHEEL_SEPARATION_LENGTH / (abs(self.drive_radius) + signed_width)
                rr = sign * -WHEEL_SEPARATION_LENGTH / (abs(self.drive_radius) - signed_width)

                if self.drive_camera_angle == 0:
                    pass
                elif self.drive_camera_angle == 9000:
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
                elif self.drive_camera_angle == -9000:
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
                else:
                    assert False, "Unsupported angle: " + str(self.drive_camera_angle)

            else: # if driving straight but camera at an angle, point all wheels in the same direction for crab movement
                angle = math.radians(self.drive_camera_angle / 100.0)
                rr = fr = fl = rl = angle

            steering = [fl, fr, rl, rr]

            # stay put while joint angles are catching up
            if self.drive_camera_angle != 0 and self.prev_position is not None and not self.in_driving_recovery:
                if (
                        (
                            abs(self.prev_position[self.joint_name.index(b'bl_steering_arm_joint')] - rl) > 0.2 or
                            abs(self.prev_position[self.joint_name.index(b'br_steering_arm_joint')] - rr) > 0.2 or
                            abs(self.prev_position[self.joint_name.index(b'fl_steering_arm_joint')] - fl) > 0.2 or
                            abs(self.prev_position[self.joint_name.index(b'fr_steering_arm_joint')] - fr) > 0.2
                        ) and (
                            self.steering_wait_start is None or
                            self.sim_time - self.steering_wait_start <= timedelta(milliseconds=1500)
                        )
                ):
                    if self.steering_wait_start is None:
                        self.steering_wait_start = self.sim_time
                        self.send_request('set_brakes 20')
                    # brake while steering angles are changing so that the robot doesn't roll away while wheels turning meanwhile
                    # use braking force 20 Nm/rad which should prevent sliding but can be overcome by motor effort
                    effort = [0.0,]*4

        if self.steering_wait_start is not None and self.sim_time - self.steering_wait_start > timedelta(milliseconds=1500):
            self.send_request('set_brakes off')
            self.steering_wait_start = None

        return steering, effort

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
