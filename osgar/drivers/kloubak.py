"""
  Driver for articulated robot Kloubak
  (https://github.com/tf-czu/kloubak)
"""

import struct
import math
import ctypes
import enum

from osgar.node import Node
from osgar.bus import BusShutdownException

#WHEEL_DISTANCE = 0.475  # m
WHEEL_DISTANCE = 0.496  # m K2, can be modified by config
CENTER_AXLE_DISTANCE = 0.348  # distance from potentiometer, can be modified by config
VESC_REPORT_FREQ = 20  # was 100  # Hz
SPEED_ENC_SCALE = (33/25)*0.25 * math.pi / (4 * 3 * 60 * VESC_REPORT_FREQ)  # scale 4x found experimentally
ENC_SCALE = (33/25)*8.0/950  # TODO proper calibration (scale for large 33" wheels, old were 25")
TURNING_ANGULAR_SPEED = math.pi/8

AD_CENTER = 416.5 # K2, can be modified by config
AD_CALIBRATION_DEG = 45  # K2, can be modified by config
AD_RANGE = -182.5  # K2, can be modified by config
MAX_JOIN_ANGLE_DEG = 68
MAX_JOIN_ANGLE = math.radians(MAX_JOIN_ANGLE_DEG) # K2, can be modified by config
AD_CENTER2 = None
AD_RANGE2 = None

DOWNDROP_TOO_LONG_RAW = 800  # in millimeters, trigger for downdrop/hole
DOWNDROP_TOO_SHORT_RAW = 350  # in millimeters, trigger for low obstacle, probably not visible by 2D lidar

CAN_ID_BUTTONS = 0x1
CAN_ID_VESC_FRONT_R = 0x91
CAN_ID_VESC_FRONT_L = 0x92
CAN_ID_VESC_REAR_R = 0x93
CAN_ID_VESC_REAR_L = 0x94
CAN_ID_VESC_REAR_K3_R = 0x95
CAN_ID_VESC_REAR_K3_L = 0x96
CAN_ID_SYNC = CAN_ID_VESC_FRONT_L
CAN_ID_CURRENT = 0x70
CAN_ID_DOWNDROPS_FRONT = 0x71
CAN_ID_DOWNDROPS_REAR = 0x72
CAN_ID_JOIN_ANGLE = 0x80
CAN_ID_REGULATED_VOLTAGE = 0x81
CAN_ID_VOLTAGE = 0x82
CAN_ID_ENCODERS = 0x83

INDEX_FRONT_LEFT = 1
INDEX_FRONT_RIGHT = 0
INDEX_REAR_LEFT = 3
INDEX_REAR_RIGHT = 2
INDEX_REAR_K3_LEFT = 5
INDEX_REAR_K3_RIGHT = 4

MIN_SPEED = 0.3

#Transferring coefficient for the vesc tachometers to meters: distance = vesc_value * 0.845/100


class DriveMode(enum.Enum):
    FRONT = 1
    REAR = 2
    ALL = 3  # 4WD


def CAN_triplet(msg_id, data):
    return [msg_id, bytes(data), 0]  # flags=0, i.e basic addressing


def draw(arr, join_arr):
    import matplotlib.pyplot as plt
    t = [a[0] for a in arr]
    values = [a[1:] for a in arr]

    f, ax = plt.subplots(2, sharex=True)

    line = ax[0].plot(t, values, '-', linewidth=2)
    ax[0].legend(line, ['request left', 'request right',
                      'enc front left', 'enc front right',
                      'enc rear left', 'enc rear right'])

    line = ax[1].plot(t, join_arr, '-', linewidth=2)

    plt.xlabel('time (s)')
    plt.show()


def draw_enc_stat(arr):
    import matplotlib.pyplot as plt
    t = [a[0] for a in arr]
    values = [a[1:] for a in arr]

    line = plt.plot(t, values, '-', linewidth=2)

    plt.xlabel('time (s)')
    plt.show()


def draw_4wd(speed_arr, join_arr):
    import numpy as np
    from matplotlib import pyplot as plt

    fig = plt.figure(figsize=(8,8))
    if len(speed_arr) > 0:
        speed_arr = np.array(speed_arr)
        print(speed_arr.shape)
        ax1 = fig.add_subplot(211)
        ax1.plot(speed_arr[:, 0], speed_arr[:, 1], "+r", label="front left")
        ax1.plot(speed_arr[:, 0], speed_arr[:, 2], "+g", label="front right")
        ax1.plot(speed_arr[:, 0], speed_arr[:, 3], "+b", label="rear left")
        ax1.plot(speed_arr[:, 0], speed_arr[:, 4], "+y", label="rear right")
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Speed (ms$^{-1}$)")
        ax1.legend()
    else:
        print("No speed data")

    if len(join_arr) > 0:
        join_arr = np.array(join_arr)
        print(join_arr.shape)
        ax2 = fig.add_subplot(212)
        ax2.plot(join_arr[:, 0], join_arr[:, 1], "+r", label="actual angle")
        ax2.plot(join_arr[:, 0], join_arr[:, 2], "+g", label="desired angle")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Angle (deg)")
        ax2.legend()
    else:
        print("No join data")

    plt.show()


def compute_desired_erpm(desired_speed, desired_angular_speed):
    scale = 1 / (VESC_REPORT_FREQ * SPEED_ENC_SCALE)
    left = scale * (desired_speed - desired_angular_speed * WHEEL_DISTANCE / 2)
    right = scale * (desired_speed + desired_angular_speed * WHEEL_DISTANCE / 2)
    return int(round(left)), int(round(right))


def compute_desired_angle(desired_speed, desired_angular_speed):
    # The angle is computed from triangle, where one
    # side has length = radius (computed from speed and angular
    # speed) and the far side is CENTER_AXLE_DISTANCE.
    # The Kloubak part is perpendicular to the turning center.
    if abs(desired_angular_speed) < 0.000001:
        return 0.0
    radius = desired_speed/desired_angular_speed
    if abs(radius) < 0.000001:
        # this case is undefined, we can only for large angle
        return math.copysign(MAX_JOIN_ANGLE, desired_angular_speed)

    desired_angle = 2 * math.atan(CENTER_AXLE_DISTANCE / radius)
    if abs(desired_angle) > MAX_JOIN_ANGLE:
        return math.copysign(MAX_JOIN_ANGLE, desired_angle)
    return desired_angle


def calculate_wheels_speeds( desired_speed, desired_angular_speed, actual_angle ):
    desired_angle = compute_desired_angle( desired_speed, desired_angular_speed)
    if abs(actual_angle) < 0.05:
        v_fl = v_rl = v_fr = v_rr = desired_speed
    else:
        actual_radius = CENTER_AXLE_DISTANCE / math.tan(actual_angle/2)
        v_fl = v_rl = desired_speed / actual_radius * (actual_radius - WHEEL_DISTANCE / 2)
        v_fr = v_rr = desired_speed / actual_radius * (actual_radius + WHEEL_DISTANCE / 2)
    if abs( desired_angle - actual_angle ) < 0.05: # in radians cca 2.9 deg
        return v_fl, v_fr, v_rl, v_rr
    turning_angular_speed = min(math.pi / 3, abs(desired_angle - actual_angle) * 1 + math.pi / 16)
#    speed_correction = CENTER_AXLE_DISTANCE * TURNING_ANGULAR_SPEED * math.tan(actual_angle / 2)
    speed_correction = CENTER_AXLE_DISTANCE * turning_angular_speed * math.tan(actual_angle/2)  # positive for left, negative for right
    turning_wheel_speed = turning_angular_speed * WHEEL_DISTANCE / 2
    if desired_angle - actual_angle > 0:  # turn left
        v_fl = v_fl - turning_wheel_speed - speed_correction
        v_fr = v_fr + turning_wheel_speed - speed_correction
        v_rl = v_rl + turning_wheel_speed + speed_correction
        v_rr = v_rr - turning_wheel_speed + speed_correction
    else:  # turn right
        v_fl = v_fl + turning_wheel_speed + speed_correction
        v_fr = v_fr - turning_wheel_speed + speed_correction
        v_rl = v_rl - turning_wheel_speed - speed_correction
        v_rr = v_rr + turning_wheel_speed - speed_correction
#    print(desired_speed, desired_angular_speed, actual_angle, v_fl, v_fr, v_rl, v_rr)
    return v_fl, v_fr, v_rl, v_rr


def joint_rad(analog, second_ang = False):
    if analog is None:
        return None
    if second_ang:
        return math.radians(AD_CALIBRATION_DEG * (AD_CENTER2 - analog) / AD_RANGE2)
    return math.radians(AD_CALIBRATION_DEG * ( AD_CENTER - analog )/AD_RANGE)


def joint_deg(analog):
    ret = joint_rad(analog)
    if ret is None:
        return None
    return math.degrees(ret)


def compute_rear(speed, angular_speed, joint_angle):
    ca, sa = math.cos(joint_angle), math.sin(joint_angle)
    x = ca * speed + sa * angular_speed * CENTER_AXLE_DISTANCE
    y = -sa * speed + ca * angular_speed * CENTER_AXLE_DISTANCE
    return x, -y


def sint16_diff(a, b):
    return ctypes.c_int16(a - b).value


def setup_global_const(config):
    global WHEEL_DISTANCE
    global CENTER_AXLE_DISTANCE
    global AD_CENTER
    global AD_CENTER2
    global AD_CALIBRATION_DEG
    global AD_RANGE
    global AD_RANGE2
    global MAX_JOIN_ANGLE
#    global TURNING_WHEEL_SPEED
    WHEEL_DISTANCE = config.get("wheel_distance", WHEEL_DISTANCE )
    CENTER_AXLE_DISTANCE = config.get("center_axle_distance", CENTER_AXLE_DISTANCE )
    AD_CENTER = config.get("ad_center", AD_CENTER )
    AD_CENTER2 = config.get("ad_center2")
    AD_CALIBRATION_DEG = config.get("ad_calibration_deg", AD_CALIBRATION_DEG )
    AD_RANGE = config.get("ad_range", AD_RANGE )
    AD_RANGE2 = config.get("ad_range2")
    MAX_JOIN_ANGLE = math.radians( config.get("max_join_angle_deg", MAX_JOIN_ANGLE_DEG) )
#    TURNING_WHEEL_SPEED = TURNING_ANGULAR_SPEED * WHEEL_DISTANCE / 2


def get_downdrop_bumpers(downdrops):
    """Create Boolean array based on raw downdrops readings"""
    return [not(DOWNDROP_TOO_SHORT_RAW < dist_mm < DOWNDROP_TOO_LONG_RAW) for dist_mm in downdrops]


class RobotKloubak(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('pose2d', 'emergency_stop', 'encoders', 'can', 'joint_angle',
                     'bumpers_front', 'bumpers_rear',
                     'downdrops_front', 'downdrops_rear')
        setup_global_const(config)

        # commands
        self.desired_speed = 0.0  # m/s
        self.desired_angular_speed = 0.0
        self.v_fl = self.v_fr = self.v_rl = self.v_rr = 0  # values in m/s
        self.drive_mode = DriveMode[config.get("drive_mode", "FRONT")]
        self.num_axis = config.get("num_axis", 2)

        # status
        self.emergency_stop = None  # unknown state
        self.pose = (0.0, 0.0, 0.0)  # x, y in meters, heading in radians (not corrected to 2PI)
        self.buttons = None
        self.last_encoders_front_left = None
        self.last_encoders_front_right = None
        self.last_encoders_rear_left = None
        self.last_encoders_rear_right = None
        self.last_encoders_rear_k3_l = None
        self.last_encoders_rear_k3_r = None
        self.last_encoders_time = None
        self.last_pose_encoders = [0, 0,] * self.num_axis  # accumulated tacho readings
        self.encoders = [0, 0,] * self.num_axis  # handling 16bit integer overflow
        self.last_encoders_16bit = None  # raw readings
        self.last_join_angle = None
        self.last_join_angle2 = None
        self.voltage = None
        self.downdrops_front = None  # array of downdrop sensors
        self.downdrops_rear = None
        self.can_errors = 0  # count errors instead of assert

        # new per axis encoders
        self.epoch = None  # unknown
        self.partial_encoders = [None, None,] * self.num_axis

        self.verbose = False  # should be in Node
        self.enc_debug_arr = []
        self.join_debug_arr = []
        self.speed_debug_arr = []
        self.count = [0, 0,] * self.num_axis
        self.count_arr = []
        self.debug_odo = []
        self.debug_downdrops_front = []
        self.debug_downdrops_rear = []
        print('Kloubak mode:', self.drive_mode, 'num_axis:', self.num_axis)

    def send_pose(self):
        x, y, heading = self.pose
        self.publish('pose2d', [round(x*1000), round(y*1000),
                                round(math.degrees(heading)*100)])

    def update_buttons(self, data):
        # assert len(data) == 1, len(data)
        if len(data) != 1:
            self.can_errors += 1
            return
        val = data[0]
        if self.buttons is None or val != self.buttons:
            self.buttons = val
            stop_status = self.buttons & 0x01 == 0x01
            if self.emergency_stop != stop_status:
                self.emergency_stop = stop_status
                self.bus.publish('emergency_stop', self.emergency_stop)
                print('Emergency STOP:', self.emergency_stop)

    def partial_axis_encoders_update(self, arr):
        # return True when completed
        # simple report encoders on epoch change -> delayed 1 cycle
        epoch, axis, left, right = arr
        ret = self.partial_encoders[:]
#        assert axis in [1, 2, 3], axis
        if axis == 0:
            print("Error: nonvalid axis: ", axis)
            return None
        self.partial_encoders[2 * (axis - 1)] = left
        self.partial_encoders[2 * (axis - 1) + 1] = right
        if self.epoch is None or self.epoch == epoch:
            self.epoch = epoch
            return None
        self.epoch = epoch
        if None in ret:
            return None  # do not publish incomplete readings
        return ret

    def update_encoders(self, msg_id, data):
        if msg_id == 0x83:
            if len(data) == 8:
                # old version used on Kloubak K2, two encoders in single message
                encoders = struct.unpack('>HHHH', data)
            elif len(data) == 6:
                # new version for Kloubak K3, one message per axis
                arr = struct.unpack('>BBHH', data)
                encoders = self.partial_axis_encoders_update(arr)
                if encoders is None:
                    return
            else:
                self.can_errors += 1
                return

            if self.last_encoders_time is not None:
                diff = [sint16_diff(e, prev) for prev, e in zip(self.last_encoders_16bit, encoders)]
            else:
                diff = [0,] * len(encoders)  # support both K2 and K3
            self.encoders = [e + d for e, d in zip(self.encoders, diff)]
            self.last_encoders_16bit = encoders
            self.last_encoders_time = self.time
            if self.verbose:
                print(self.time, diff, self.encoders)
            return encoders
        assert msg_id in [0x91, 0x92, 0x93, 0x94, 0x95, 0x96], hex(msg_id)
        # assert len(data) == 8, data
        if len(data) != 8:
            self.can_errors += 1
            return
        rpm3, current, duty_cycle = struct.unpack('>ihh', data)
        if msg_id == CAN_ID_VESC_FRONT_L:
            self.last_encoders_front_left = rpm3
#            print('left', rpm3, current, duty_cycle)
        elif msg_id == CAN_ID_VESC_FRONT_R:
            self.last_encoders_front_right = rpm3
#            print('right', rpm3, current, duty_cycle)
        if msg_id == CAN_ID_VESC_REAR_L:
            self.last_encoders_rear_left = rpm3
        elif msg_id == CAN_ID_VESC_REAR_R:
            self.last_encoders_rear_right = rpm3
        if msg_id == CAN_ID_VESC_REAR_K3_L:
            self.last_encoders_rear_k3_l = rpm3
        elif msg_id == CAN_ID_VESC_REAR_K3_R:
            self.last_encoders_rear_k3_r = rpm3
        self.count[msg_id - 0x91] += 1
#        print(self.count)
        if self.verbose:
            self.count_arr.append([self.time.total_seconds()] + self.count)

    def compute_pose(self, left, right):
        """Update internal pose with 'dt' step"""
        if left is None or right is None:
            return False, None, None
        x, y, heading = self.pose

        metricL = ENC_SCALE * left  # dt is already part of ENC_SCALE
        metricR = ENC_SCALE * right

        dist = (metricL + metricR)/2.0
        angle = (metricR - metricL)/WHEEL_DISTANCE

        # advance robot by given distance and angle
        if abs(angle) < 0.0000001:  # EPS
            # Straight movement - a special case
            x += dist * math.cos(heading)
            y += dist * math.sin(heading)
            #Not needed: heading += angle
        else:
            # Arc
            r = dist / angle
            x += -r * math.sin(heading) + r * math.sin(heading + angle)
            y += +r * math.cos(heading) - r * math.cos(heading + angle)
            heading += angle # not normalized
        return True, (x, y, heading), (dist, angle)

    def update_pose(self):
        diff = [e - prev for e, prev in zip(self.encoders, self.last_pose_encoders)]
        self.last_pose_encoders = self.encoders
        if self.num_axis == 2:
            if self.desired_speed >= 0:
                ret, pose, motion = self.compute_pose(diff[INDEX_REAR_LEFT], diff[INDEX_REAR_RIGHT])
            else:
                ret, pose, motion = self.compute_pose(diff[INDEX_FRONT_LEFT], diff[INDEX_FRONT_RIGHT])
        elif self.num_axis == 3:
            if self.desired_speed >= 0:
                ret, pose, motion = self.compute_pose(diff[INDEX_REAR_K3_LEFT], diff[INDEX_REAR_K3_RIGHT])
            else:
                ret, pose, motion = self.compute_pose(diff[INDEX_FRONT_LEFT], diff[INDEX_FRONT_RIGHT])
        else:
            assert False, self.num_axis  # 2 or 3 are supported only

        if ret:
            self.pose = pose

        if self.verbose and ret and self.last_join_angle is not None:
#            self.debug_odo.append((self.time.total_seconds(), motion[0], motion_rear[0]))
#            self.debug_odo.append((self.time.total_seconds(), motion[1], motion_rear[1]))
#            self.debug_odo.append((self.time.total_seconds(), motion[0], motion[1],
#                                   motion_rear[0], motion_rear[1],
#                                   joint_deg(self.last_join_angle)))
            estimate = compute_rear(motion[0], motion[1], joint_rad(self.last_join_angle))
#            self.debug_odo.append((self.time.total_seconds(), motion[0], motion_rear[0], estimate[0]))
#            self.debug_odo.append((self.time.total_seconds(), motion[1], motion_rear[1], estimate[1]))
#            self.debug_odo.append((self.time.total_seconds(), motion_rear[1], estimate[1]))
        return ret

    def process_packet(self, packet, verbose=False):
        msg_id, payload, flags = packet
        if msg_id == CAN_ID_BUTTONS:
            self.update_buttons(payload)
        elif msg_id == CAN_ID_CURRENT:
            # expected 24bit integer miliAmps
            if len(payload) == 3:
                current = struct.unpack('>i', b'\x00' + payload)[0]
#                print(current)
            else:
                self.can_errors += 1
        elif msg_id == CAN_ID_JOIN_ANGLE:
            if len(payload) == 4:
                if self.num_axis == 3:
                    self.last_join_angle, self.last_join_angle2 = struct.unpack('>hh', payload)
    #                print(self.last_join_angle,payload)
#                    print(self.last_join_angle, self.last_join_angle2)
                    angle1, angle2 = joint_rad(self.last_join_angle), joint_rad(self.last_join_angle2, second_ang=True)
                    self.publish('joint_angle', [round(math.degrees(angle1)*100), round(math.degrees(angle2)*100)])
                else:
                    self.last_join_angle = struct.unpack('>i', payload)[0]
#                    print(self.last_join_angle,payload)
#                    print(self.last_join_angle)
                    angle = joint_rad(self.last_join_angle)
                    self.publish('joint_angle', [round(math.degrees(angle)*100)])
            else:
                self.can_errors += 1
        elif msg_id == CAN_ID_VOLTAGE:
            if len(payload) == 4:
                self.voltage = struct.unpack_from('>i', payload)[0]
#                print(self.voltage)
            else:
                self.can_errors += 1
        elif msg_id == CAN_ID_DOWNDROPS_FRONT:
            # assert len(packet) == 2 + 4, len(packet)
            if len(payload) == 4:
                self.downdrops_front = struct.unpack_from('>HH', payload)
                self.publish('downdrops_front', list(self.downdrops_front))
                self.publish('bumpers_front', get_downdrop_bumpers(self.downdrops_front))
#                print(self.time, self.downdrops_front)
                if self.verbose:
                    self.debug_downdrops_front.append(
                            (self.time.total_seconds(), self.downdrops_front[0], self.downdrops_front[1]))
            else:
                self.can_errors += 1
        elif msg_id == CAN_ID_DOWNDROPS_REAR:
            # assert len(packet) == 2 + 4, len(packet)
            if len(payload) == 4:
                self.downdrops_rear = struct.unpack_from('>HH', payload)
                self.publish('downdrops_rear', list(self.downdrops_rear))
                self.publish('bumpers_rear', get_downdrop_bumpers(self.downdrops_rear))
#                print(self.time, self.downdrops_rear)
                if self.verbose:
                    self.debug_downdrops_rear.append(
                            (self.time.total_seconds(), self.downdrops_rear[0], self.downdrops_rear[1]))
            else:
                self.can_errors += 1
        elif msg_id in [CAN_ID_ENCODERS, CAN_ID_VESC_FRONT_L, CAN_ID_VESC_FRONT_R, CAN_ID_VESC_REAR_L, CAN_ID_VESC_REAR_R, CAN_ID_VESC_REAR_K3_R, CAN_ID_VESC_REAR_K3_L]:
            if self.update_encoders(msg_id, payload) is not None:
                diff = [e - prev for e, prev in zip(self.encoders, self.last_pose_encoders)]
                # note, that this craziness is necessary only because the motor indexes
                # are assigned first right and then left
                if len(diff) == 4:
                    self.publish('encoders',
                            [diff[INDEX_FRONT_LEFT], diff[INDEX_FRONT_RIGHT],
                             diff[INDEX_REAR_LEFT], diff[INDEX_REAR_RIGHT]])
                else:
                    assert len(diff) == 6, len(diff)  # Kloubak K3
                    self.publish('encoders',
                            [diff[INDEX_FRONT_LEFT], diff[INDEX_FRONT_RIGHT],
                             diff[INDEX_REAR_LEFT], diff[INDEX_REAR_RIGHT],
                             diff[INDEX_REAR_K3_LEFT], diff[INDEX_REAR_K3_RIGHT]])
                if self.update_pose():
                    self.send_pose()
                return True
        return False


    def update_drive_three_axles(self, timestamp, data):
        if self.last_join_angle and self.desired_speed > 0:
            angle = joint_rad(self.last_join_angle)
            assert abs(angle) < math.pi/2, angle * 180 / math.pi
            self.v_fl, self.v_fr, self.v_rl, self.v_rr = calculate_wheels_speeds(self.desired_speed,
                                                                                 self.desired_angular_speed, angle)
            if self.verbose:
                self.join_debug_arr.append([timestamp.total_seconds(), angle*180/math.pi, compute_desired_angle( self.desired_speed, self. desired_angular_speed )*180/math.pi] )

        elif self.last_join_angle2 and self.desired_speed < 0:
            angle = joint_rad(self.last_join_angle2, second_ang = True)
            assert abs(angle) < math.pi / 2, angle * 180 / math.pi
            self.v_fl, self.v_fr, self.v_rl, self.v_rr = calculate_wheels_speeds(self.desired_speed,
                                                                                 self.desired_angular_speed, angle)
            if self.verbose:
                self.join_debug_arr.append([timestamp.total_seconds(), angle*180/math.pi, compute_desired_angle( self.desired_speed, self. desired_angular_speed )*180/math.pi] )

        if self.process_packet(data):
            stop = [0, 0, 0, 0]

            if self.verbose:
                self.speed_debug_arr.append( [ timestamp.total_seconds(), self.v_fl, self.v_fr, self.v_rl, self.v_rr ] )

            if self.desired_speed > 0:
                if self.last_encoders_front_right is not None:
                    v_fr_2 = int(round(self.v_fr / (VESC_REPORT_FREQ * SPEED_ENC_SCALE)))
                    self.publish('can', CAN_triplet(0x31, list(struct.pack('>i', v_fr_2))))  # front right
                if self.last_encoders_front_left is not None:
                    v_fl_2 = int(round(self.v_fl / (VESC_REPORT_FREQ * SPEED_ENC_SCALE)))
                    self.publish('can', CAN_triplet(0x32, list(struct.pack('>i', v_fl_2))))  # front left
                if self.last_encoders_rear_right is not None:
                    v_rr_2 = int(round(self.v_rr / (VESC_REPORT_FREQ * SPEED_ENC_SCALE)))
                    self.publish('can', CAN_triplet(0x33, list(struct.pack('>i', v_rr_2))))  # right mid
                if self.last_encoders_rear_left is not None:
                    v_rl_2 = int(round(self.v_rl / (VESC_REPORT_FREQ * SPEED_ENC_SCALE)))
                    self.publish('can', CAN_triplet(0x34, list(struct.pack('>i', v_rl_2))))  # left mid
                self.publish('can', CAN_triplet(0x15, stop))  # right rear
                self.publish('can', CAN_triplet(0x16, stop))  # left rear

            elif self.desired_speed < 0:
                self.publish('can', CAN_triplet(0x11, stop))  # right front
                self.publish('can', CAN_triplet(0x12, stop))  # left front

                if self.last_encoders_rear_right is not None:
                    v_fr_2 = int(round(self.v_fr / (VESC_REPORT_FREQ * SPEED_ENC_SCALE)))
                    self.publish('can', CAN_triplet(0x33, list(struct.pack('>i', v_fr_2))))  # mid right
                if self.last_encoders_rear_left is not None:
                    v_fl_2 = int(round(self.v_fl / (VESC_REPORT_FREQ * SPEED_ENC_SCALE)))
                    self.publish('can', CAN_triplet(0x34, list(struct.pack('>i', v_fl_2))))  # mid left
                if self.last_encoders_rear_k3_r is not None:
                    v_rr_2 = int(round(self.v_rr / (VESC_REPORT_FREQ * SPEED_ENC_SCALE)))
                    self.publish('can', CAN_triplet(0x35, list(struct.pack('>i', v_rr_2))))  # right rear
                if self.last_encoders_rear_k3_l is not None:
                    v_rl_2 = int(round(self.v_rl / (VESC_REPORT_FREQ * SPEED_ENC_SCALE)))
                    self.publish('can', CAN_triplet(0x36, list(struct.pack('>i', v_rl_2))))  # left rear

            else:
                self.publish('can', CAN_triplet(0x11, stop))  # right front
                self.publish('can', CAN_triplet(0x12, stop))  # left front
                self.publish('can', CAN_triplet(0x13, stop))  # right mid
                self.publish('can', CAN_triplet(0x14, stop))  # left mid
                self.publish('can', CAN_triplet(0x15, stop))  # right rear
                self.publish('can', CAN_triplet(0x16, stop))  # left rear


    def update_drive_two_axles(self, timestamp, data):
        """Drive all motors"""
        # update commands for all 4 motors only with new angle reading
        # (Kloubak does not have any SYNC signal for all motor drivers)
        # TODO integration front driver?
        if self.last_join_angle:
            angle = joint_rad(self.last_join_angle)
            assert abs(angle) < math.pi / 2, angle * 180 / math.pi
            self.v_fl, self.v_fr, self.v_rl, self.v_rr = calculate_wheels_speeds(self.desired_speed,
                                                                                 self.desired_angular_speed, angle)
            if self.verbose:
                self.join_debug_arr.append([timestamp.total_seconds(), angle*180/math.pi, compute_desired_angle( self.desired_speed, self. desired_angular_speed )*180/math.pi] )

        if self.process_packet(data):
            stop = [0, 0, 0, 0]
#
            if self.verbose:
                self.speed_debug_arr.append( [ timestamp.total_seconds(), self.v_fl, self.v_fr, self.v_rl, self.v_rr ] )

            if self.desired_speed != 0: # and self.drive_mode == DriveMode.ALL: ?
                if self.last_encoders_front_right is not None:
                    v_fr_2 = int(round(self.v_fr / (VESC_REPORT_FREQ * SPEED_ENC_SCALE)))
                    self.publish('can', CAN_triplet(0x31, list(struct.pack('>i', v_fr_2))))  # front right
                if self.last_encoders_front_left is not None:
                    v_fl_2 = int(round(self.v_fl / (VESC_REPORT_FREQ * SPEED_ENC_SCALE)))
                    self.publish('can', CAN_triplet(0x32, list(struct.pack('>i', v_fl_2))))  # front left
                if self.last_encoders_rear_right is not None:
                    v_rr_2 = int(round(self.v_rr / (VESC_REPORT_FREQ * SPEED_ENC_SCALE)))
                    self.publish('can', CAN_triplet(0x33, list(struct.pack('>i', v_rr_2))))  # rear right
                if self.last_encoders_rear_left is not None:
                    v_rl_2 = int(round(self.v_rl / (VESC_REPORT_FREQ * SPEED_ENC_SCALE)))
                    self.publish('can', CAN_triplet(0x34, list(struct.pack('>i', v_rl_2))))  # rear left

#            elif self.drive_mode == DriveMode.FRONT ?

            else:
                self.publish('can', CAN_triplet(0x11, stop))  # right front
                self.publish('can', CAN_triplet(0x12, stop))  # left front
                self.publish('can', CAN_triplet(0x13, stop))  # right rear
                self.publish('can', CAN_triplet(0x14, stop))  # left rear


    def update_drive_front_wheels(self, timestamp, data):
        send_speed = self.desired_speed
        if self.last_join_angle is not None and abs(self.desired_speed) > MIN_SPEED:
            desired_angle = compute_desired_angle(self.desired_speed, self.desired_angular_speed)
            angle = joint_rad(self.last_join_angle)
            angle_diff = abs( desired_angle - angle )
            if angle_diff > 0.2 and self.desired_speed > 0:
                send_speed = max(MIN_SPEED, self.desired_speed * (1.2 - angle_diff))  # 1 rad is 57 deg
            elif angle_diff > 0.2 and self.desired_speed < 0:
                send_speed = min(- MIN_SPEED, self.desired_speed * (1.2 - angle_diff))  # 1 rad is 57 deg

        limit_l, limit_r = compute_desired_erpm(send_speed, self.desired_angular_speed)
#        limit_l, limit_r = compute_desired_erpm(self.desired_speed, self.desired_angular_speed)
        if self.process_packet(data):
            stop = [0, 0, 0, 0]
            if self.verbose:
                cmd_l, cmd_r = limit_l, limit_r
                if self.desired_speed < 0:
                    cmd_l, cmd_r = -cmd_l, -cmd_r
                elif self.desired_speed == 0:
                    cmd_l, cmd_r = 0, 0
                self.enc_debug_arr.append((timestamp.total_seconds(), cmd_l, cmd_r,
                    self.last_encoders_front_left, self.last_encoders_front_right,
                    self.last_encoders_rear_left, self.last_encoders_rear_right))
                self.join_debug_arr.append(joint_deg(self.last_join_angle))

            rear_drive = False  # True  # experimental
            if rear_drive and self.desired_speed > 0:
                # control the joint angle and the desired speed
                self.publish('can', CAN_triplet(0x11, stop))  # right front
                self.publish('can', CAN_triplet(0x12, stop))  # left front

                if self.last_join_angle is not None:
                    desired_angle = compute_desired_angle(self.desired_speed, self.desired_angular_speed)
                    angle = joint_rad(self.last_join_angle)
                    diff = abs(desired_angle - angle)
                    speed = self.desired_speed * (1 - diff/math.radians(80))

                    angular_speed = desired_angle - angle  # i.e. in 1s it should be the same
                    limit_l, limit_r = compute_desired_erpm(speed, -angular_speed)  # mirror flip (rear)

                if self.last_encoders_rear_right is not None:
                    self.publish('can', CAN_triplet(0x33, list(struct.pack('>i', limit_r))))
                if self.last_encoders_rear_left is not None:
                    self.publish('can', CAN_triplet(0x34, list(struct.pack('>i', limit_l))))

            elif self.desired_speed > 0:
                if self.last_encoders_front_right is not None:
                    self.publish('can', CAN_triplet(0x31, list(struct.pack('>i', limit_r))))
                if self.last_encoders_front_left is not None:
                    self.publish('can', CAN_triplet(0x32, list(struct.pack('>i', limit_l))))
                self.publish('can', CAN_triplet(0x13, stop))  # rear right
                self.publish('can', CAN_triplet(0x14, stop))  # rear left

            elif self.desired_speed < 0:
                if self.last_encoders_rear_right is not None:
                    self.publish('can', CAN_triplet(0x33, list(struct.pack('>i', limit_r))))
                if self.last_encoders_rear_left is not None:
                    self.publish('can', CAN_triplet(0x34, list(struct.pack('>i', limit_l))))
                self.publish('can', CAN_triplet(0x11, stop))  # right front
                self.publish('can', CAN_triplet(0x12, stop))  # left front

            else:
                self.publish('can', CAN_triplet(0x11, stop))  # right front
                self.publish('can', CAN_triplet(0x12, stop))  # left front
                self.publish('can', CAN_triplet(0x13, stop))  # right rear
                self.publish('can', CAN_triplet(0x14, stop))  # left rear


    def slot_can(self, timestamp, data):
        self.time = timestamp
        if self.num_axis == 3:
            self.update_drive_three_axles(timestamp, data)

        elif self.num_axis == 2:
            if self.drive_mode == DriveMode.ALL:
                self.update_drive_two_axles(timestamp, data)
            elif self.drive_mode == DriveMode.FRONT:
                self.update_drive_front_wheels(timestamp, data)
            else:
                assert False, self.drive_mode  # is not supported yet

        else:
            assert False, self.num_axis  # 2 or 3 are supported only


    def slot_desired_speed(self, timestamp, data):
        self.desired_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)

    def run(self):
        try:
            while True:
                try:
                    self.time, channel, data = self.listen()
                except StopIteration:
                    if self.verbose:
                        print(len(self.enc_debug_arr))
                    raise BusShutdownException

                if channel == 'can':
                    self.slot_can(self.time, data)
                elif channel == 'desired_speed':
                    self.slot_desired_speed(self.time, data)
                else:
                    assert False, channel  # unsupported channel
        except BusShutdownException:
            pass
        assert self.can_errors == 0, self.can_errors  # summary over whole run

    def draw(self):
        """
        Debug - draw encoders
        """
#        draw(self.enc_debug_arr, self.join_debug_arr)
#        print(self.count_arr)
#        print(self.count_arr[-1])
#        draw_enc_stat(self.count_arr)
#        draw_enc_stat(self.debug_odo)
        draw_4wd( self.speed_debug_arr, self.join_debug_arr )
#        draw_enc_stat(self.debug_downdrops_front)
#        draw_enc_stat(self.debug_downdrops_rear)

# vim: expandtab sw=4 ts=4
