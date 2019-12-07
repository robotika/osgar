"""
  Driver for articulated robot Kloubak
  (https://github.com/tf-czu/kloubak)
"""

import struct
import math
import ctypes

from osgar.node import Node
from osgar.bus import BusShutdownException

#WHEEL_DISTANCE = 0.475  # m
WHEEL_DISTANCE = 0.496  # m K2
CENTER_AXLE_DISTANCE = 0.348  # distance from potentiometer
VESC_REPORT_FREQ = 20  # was 100  # Hz
SPEED_ENC_SCALE = (33/25)*0.25 * math.pi / (4 * 3 * 60 * VESC_REPORT_FREQ)  # scale 4x found experimentally
ENC_SCALE = (33/25)*8.0/950  # TODO proper calibration (scale for large 33" wheels, old were 25")

AD_CENTER = 419.7 # K2
AD_MAX_DEG = 45  # K2
AD_RANGE = -182.5  # K2
AD_HW_LIMIT_LEFT = 12480  # corresponds to circle 37cm of touching left wheels
AD_HW_LIMIT_RIGHT = 3584  # circle 40cm diameter, touching right wheels

CAN_ID_BUTTONS = 0x1
CAN_ID_VESC_FRONT_R = 0x91
CAN_ID_VESC_FRONT_L = 0x92
CAN_ID_VESC_REAR_R = 0x93
CAN_ID_VESC_REAR_L = 0x94
CAN_ID_SYNC = CAN_ID_VESC_FRONT_L
CAN_ID_CURRENT = 0x70
CAN_ID_JOIN_ANGLE = 0x80
CAN_ID_REGULATED_VOLTAGE = 0x81
CAN_ID_VOLTAGE = 0x82
CAN_ID_ENCODERS = 0x83

INDEX_FRONT_LEFT = 1
INDEX_FRONT_RIGHT = 0
INDEX_REAR_LEFT = 3
INDEX_REAR_RIGHT = 2

MIN_SPEED = 0.3

#Transferring coefficient for the vesc tachometers to meters: distance = vesc_value * 0.845/100


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


def compute_desired_erpm(desired_speed, desired_angular_speed):
    scale = 1 / (VESC_REPORT_FREQ * SPEED_ENC_SCALE)
    left = scale * (desired_speed - desired_angular_speed * WHEEL_DISTANCE / 2)
    right = scale * (desired_speed + desired_angular_speed * WHEEL_DISTANCE / 2)
    return int(round(left)), int(round(right))


def compute_desired_angle(desired_speed, desired_angular_speed):
    # The angle is computed from triangle, where one
    # side has length = radius (computed from speed and angular
    # speed) and the far side is half of CENTER_AXLE_DISTANCE.
    # The Kloubak part is perpendicular to the turning center.
    if abs(desired_angular_speed) < 0.000001:
        return 0.0
    radius = desired_speed/desired_angular_speed
    if abs(2 * radius) > CENTER_AXLE_DISTANCE:
        return 2 * math.asin((CENTER_AXLE_DISTANCE/2) / radius)
    return math.pi if radius > 0 else -math.pi


def joint_rad(analog):
    if analog is None:
        return None
    return math.radians(AD_MAX_DEG * ( AD_CENTER - analog )/AD_RANGE)


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


class RobotKloubak(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('pose2d', 'emergency_stop', 'encoders', 'can')

        # commands
        self.desired_speed = 0.0  # m/s
        self.desired_angular_speed = 0.0

        # status
        self.emergency_stop = None  # uknown state
        self.pose = (0.0, 0.0, 0.0)  # x, y in meters, heading in radians (not corrected to 2PI)
        self.buttons = None
        self.last_encoders_front_left = None
        self.last_encoders_front_right = None
        self.last_encoders_rear_left = None
        self.last_encoders_rear_right = None
        self.last_encoders_time = None
        self.last_pose_encoders = [0, 0, 0, 0]  # accumulated tacho readings
        self.encoders = [0, 0, 0, 0]  # handling 16bit integrer overflow
        self.last_encoders_16bit = None  # raw readings
        self.last_join_angle = None
        self.voltage = None
        self.can_errors = 0  # count errors instead of assert

        self.verbose = False  # should be in Node
        self.enc_debug_arr = []
        self.join_debug_arr = []
        self.count = [0, 0, 0, 0]
        self.count_arr = []
        self.debug_odo = []

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

    def update_encoders(self, msg_id, data):
        if msg_id == 0x83:
            encoders = struct.unpack('>HHHH', data)
            if self.last_encoders_time is not None:
                diff = [sint16_diff(e, prev) for prev, e in zip(self.last_encoders_16bit, encoders)]
            else:
                diff = [0, 0, 0, 0]
            self.encoders = [e + d for e, d in zip(self.encoders, diff)]
            self.last_encoders_16bit = encoders
            self.last_encoders_time = self.time
            if self.verbose:
                print(self.time - self.last_encoders_time, diff, self.encoders)
            return
        assert msg_id in [0x91, 0x92, 0x93, 0x94], hex(msg_id)
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
        if self.desired_speed >= 0:
            ret, pose, motion = self.compute_pose(diff[INDEX_REAR_LEFT], diff[INDEX_REAR_RIGHT])
        else:
            ret, pose, motion = self.compute_pose(diff[INDEX_FRONT_LEFT], diff[INDEX_FRONT_RIGHT])

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
        elif msg_id in [CAN_ID_ENCODERS, CAN_ID_VESC_FRONT_L, CAN_ID_VESC_FRONT_R, CAN_ID_VESC_REAR_L, CAN_ID_VESC_REAR_R]:
            self.update_encoders(msg_id, payload)
        elif msg_id == CAN_ID_CURRENT:
            # expected 24bit integer miliAmps
            if len(payload) == 3:
                current = struct.unpack('>i', b'\x00' + payload)[0]
#                print(current)
            else:
                self.can_errors += 1
        elif msg_id == CAN_ID_JOIN_ANGLE:
            if len(payload) == 4:
                self.last_join_angle = struct.unpack('>i', payload)[0]
#                print(self.last_join_angle)
            else:
                self.can_errors += 1
        elif msg_id == CAN_ID_VOLTAGE:
            if len(payload) == 4:
                self.voltage = struct.unpack_from('>i', payload)[0]
#                print(self.voltage)
            else:
                self.can_errors += 1
        if msg_id == CAN_ID_ENCODERS:
            diff = [e - prev for e, prev in zip(self.encoders, self.last_pose_encoders)]
            self.publish('encoders',
                    [diff[INDEX_FRONT_LEFT], diff[INDEX_FRONT_RIGHT],
                     diff[INDEX_REAR_LEFT], diff[INDEX_REAR_RIGHT]])
            if self.update_pose():
                self.send_pose()

                # reset all encoder values to be sure that new reading were received
#                self.last_encoders_front_left = None
#                self.last_encoders_front_right = None
#                self.last_encoders_rear_left = None
#                self.last_encoders_rear_right = None
            return True
        return False

    def slot_can(self, timestamp, data):
        self.time = timestamp

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
                    speed = self.desired_speed * (1 - diff/math.radians(AD_MAX_DEG))

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
                self.publish('can', CAN_triplet(0x13, stop))  # right rear
                self.publish('can', CAN_triplet(0x14, stop))  # left rear

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
        draw(self.enc_debug_arr, self.join_debug_arr)
#        print(self.count_arr)
        print(self.count_arr[-1])
#        draw_enc_stat(self.count_arr)
#        draw_enc_stat(self.debug_odo)

# vim: expandtab sw=4 ts=4
