"""
  Driver for articulated robot Kloubak
  (https://github.com/tf-czu/kloubak)
"""

import struct
import math

from .canserial import CAN_packet
from osgar.node import Node
from osgar.bus import BusShutdownException

#WHEEL_DISTANCE = 0.475  # m
WHEEL_DISTANCE = 0.496  # m K2
CENTER_AXLE_DISTANCE = 0.348  # distance from potentiometer
VESC_REPORT_FREQ = 20  # was 100  # Hz
ENC_SCALE = 0.25 * math.pi / (4 * 3 * 60 * VESC_REPORT_FREQ)  # scale 4x found experimentally

AD_CENTER = 404 # K2
AD_MAX_DEG = 79  # K2
AD_RANGE = 322  # K2
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
    scale = 1 / (VESC_REPORT_FREQ * ENC_SCALE)
    left = scale * (desired_speed - desired_angular_speed * WHEEL_DISTANCE / 2)
    right = scale * (desired_speed + desired_angular_speed * WHEEL_DISTANCE / 2)
    return int(round(left)), int(round(right))


def compute_desired_angle(desired_speed, desired_angular_speed):
    # The angle is computed from triangle, where one
    # side has length = radius (computed from speed and angular
    # speed) and the far side is half of CENTER_AXLE_DISTANCE.
    # The Kloubak part is perpendicular to the turning center.
#    print(desired_speed, desired_angular_speed)
    if abs(desired_angular_speed) < 0.000001:
        return 0.0
    radius = desired_speed/desired_angular_speed
#    print(radius)
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


class RobotKloubak(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)

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
        self.last_join_angle = None

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
        assert len(data) == 1, len(data)
        val = data[0]
        if self.buttons is None or val != self.buttons:
            self.buttons = val
            stop_status = self.buttons & 0x01 == 0x01
            if self.emergency_stop != stop_status:
                self.emergency_stop = stop_status
                self.bus.publish('emergency_stop', self.emergency_stop)
                print('Emergency STOP:', self.emergency_stop)

    def update_encoders(self, msg_id, data):
        assert len(data) == 8, data
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
        ret, pose, motion = self.compute_pose(self.last_encoders_front_left, self.last_encoders_front_right)
        if ret:
            self.pose = pose
        ret2, pose2, motion_rear = self.compute_pose(self.last_encoders_rear_left, self.last_encoders_rear_right)
        if self.verbose and ret and ret2 and self.last_join_angle is not None:
#            self.debug_odo.append((self.time.total_seconds(), motion[0], motion_rear[0]))
#            self.debug_odo.append((self.time.total_seconds(), motion[1], motion_rear[1]))
#            self.debug_odo.append((self.time.total_seconds(), motion[0], motion[1],
#                                   motion_rear[0], motion_rear[1],
#                                   joint_deg(self.last_join_angle)))
            estimate = compute_rear(motion[0], motion[1], joint_rad(self.last_join_angle))
#            self.debug_odo.append((self.time.total_seconds(), motion[0], motion_rear[0], estimate[0]))
#            self.debug_odo.append((self.time.total_seconds(), motion[1], motion_rear[1], estimate[1]))
            self.debug_odo.append((self.time.total_seconds(), motion_rear[1], estimate[1]))
        return ret

    def process_packet(self, packet, verbose=False):
        if len(packet) >= 2:
            msg_id = ((packet[0]) << 3) | (((packet[1]) >> 5) & 0x1f)
#            print(hex(msg_id), packet[2:])
            if msg_id == CAN_ID_BUTTONS:
                self.update_buttons(packet[2:])
            elif msg_id in [CAN_ID_VESC_FRONT_L, CAN_ID_VESC_FRONT_R, CAN_ID_VESC_REAR_L, CAN_ID_VESC_REAR_R]:
                self.update_encoders(msg_id, packet[2:])
            elif msg_id == CAN_ID_CURRENT:
                assert len(packet) == 5, len(packet)  # expected 24bit integer miliAmps
                current = struct.unpack_from('>i', packet, 1)[0] & 0xFFFFFF
#                print(current)
            elif msg_id == CAN_ID_JOIN_ANGLE:
                assert len(packet) == 2 + 4, len(packet)
                self.last_join_angle = struct.unpack_from('>i', packet, 2)[0]
#                print(self.last_join_angle)

            if msg_id == CAN_ID_SYNC:
                self.publish('encoders', 
                        [self.last_encoders_front_left, self.last_encoders_front_right,
                         self.last_encoders_rear_left, self.last_encoders_rear_right])
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
        limit_l, limit_r = compute_desired_erpm(self.desired_speed, self.desired_angular_speed)
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
                self.join_debug_arr.append(self.last_join_angle)

            rear_drive = False  # True  # experimental
            if rear_drive and self.desired_speed > 0:
                # control the joint angle and the desired speed
                self.publish('can', CAN_packet(0x11, stop))  # right front
                self.publish('can', CAN_packet(0x12, stop))  # left front

                if self.last_join_angle is not None:
                    desired_angle = compute_desired_angle(self.desired_speed, self.desired_angular_speed)
                    angle = joint_rad(self.last_join_angle)
                    diff = abs(desired_angle - angle)
                    speed = self.desired_speed * (1 - diff/math.radians(AD_MAX_DEG))

                    angular_speed = desired_angle - angle  # i.e. in 1s it should be the same
                    limit_l, limit_r = compute_desired_erpm(speed, -angular_speed)  # mirror flip (rear)

                if self.last_encoders_rear_right is not None:
                    self.publish('can', CAN_packet(0x33, list(struct.pack('>i', limit_r))))
                if self.last_encoders_rear_left is not None:
                    self.publish('can', CAN_packet(0x34, list(struct.pack('>i', limit_l))))

            elif self.desired_speed > 0:
                if self.last_encoders_front_right is not None:
                    self.publish('can', CAN_packet(0x31, list(struct.pack('>i', limit_r))))
                if self.last_encoders_front_left is not None:
                    self.publish('can', CAN_packet(0x32, list(struct.pack('>i', limit_l))))
                self.publish('can', CAN_packet(0x13, stop))  # right rear
                self.publish('can', CAN_packet(0x14, stop))  # left rear

            elif self.desired_speed < 0:
                if self.last_encoders_rear_right is not None:
                    self.publish('can', CAN_packet(0x33, list(struct.pack('>i', limit_r))))
                if self.last_encoders_rear_left is not None:
                    self.publish('can', CAN_packet(0x34, list(struct.pack('>i', limit_l))))
                self.publish('can', CAN_packet(0x11, stop))  # right front
                self.publish('can', CAN_packet(0x12, stop))  # left front 

            else:
                self.publish('can', CAN_packet(0x11, stop))  # right front
                self.publish('can', CAN_packet(0x12, stop))  # left front
                self.publish('can', CAN_packet(0x13, stop))  # right rear
                self.publish('can', CAN_packet(0x14, stop))  # left rear


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
