"""
  Matty - 4-wheel drive articulated robot with passive joint
"""
# https://robotika.vosrk.cz/robots/matty/
# https://robotika.cz/robots/matty-twins/

import math
import struct
import logging
import datetime
from enum import Enum

from osgar.node import Node


FRONT_REAR_AXIS_DISTANCE = 0.32  # meters, distance for straight motion

SYNC = 0x55
ESC = 0x56

def add_esc_chars(data):
    ret = []
    for d in data:
        if d in [ESC, SYNC]:
            ret.extend([ESC, 0xFF & (~d)])
        else:
            ret.append(d)
    return bytes(ret)


class RobotStatus(Enum):
    EMERGENCY_STOP  = 0x01
    VOLTAGE_LOW     = 0x02
    BUMPER_FRONT    = 0x04
    BUMPER_BACK     = 0x08
    ERROR_ENCODER   = 0x10
    ERROR_POWER     = 0x20
    RUNNING         = 0x80


def remove_esc_chars(data):
    """Undo excape characters to the end of data. Keep SYNC characters"""
    ret = bytes()
    esc_i = data.find(ESC)
    while esc_i >= 0:
        ret += data[:esc_i]
        assert esc_i + 1 < len(data)  # what to do with ESC at the end?!
        orig = 0xFF & (~data[esc_i + 1])
        if orig not in [ESC, SYNC]:
            logging.warning(f'Esc error {hex(orig)}'),
        ret += bytes([orig])
        data = data[esc_i + 2:]
        esc_i = data.find(ESC)
    ret += data
    return ret


class Matty(Node):

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('esp_data', 'emergency_stop', 'pose2d')
        self.max_speed = config.get('max_speed', 0.5)
        self.max_steering_deg = config.get('max_steering_deg', 45.0)
        self.pose = 0, 0, 0

        self.desired_speed = 0  # m/s
        self.desired_steering_angle_deg = 0.0  # degrees
        self.debug_arr = []
        self.verbose = False
        self.counter = 0
        self.buf = b''
        self.odometry_requested = False
        self.last_bumpers = None

    def parse_odometry(self, data):
        counter, cmd, status, mode, voltage_mV, current_mA, speed_mms, angle_deg = struct.unpack_from('<BBBBHHhh', data)
        enc = struct.unpack_from('<HHHH', data, 12)
        if (status & RobotStatus.ERROR_ENCODER.value) and (status & RobotStatus.ERROR_POWER.value) == 0:
            print(self.time, 'Status ERROR_ENCODER', hex(status))
        bumpers = status & (RobotStatus.BUMPER_BACK.value | RobotStatus.BUMPER_FRONT.value)
        if self.last_bumpers != bumpers:
            print(self.time, 'Bumpers:',
                  'front' if bumpers & RobotStatus.BUMPER_FRONT.value else '',
                  'back' if bumpers & RobotStatus.BUMPER_BACK.value else '')
            self.last_bumpers = bumpers
        if self.verbose:
            print(self.time, counter, cmd, status, mode, voltage_mV, current_mA, speed_mms, angle_deg, enc)
            self.debug_arr.append([self.time.total_seconds(), enc])
        return speed_mms/1000, math.radians(angle_deg/100)

    def publish_pose2d(self, speed, joint_angle):
        dt = 0.1  # 10Hz - maybe use real-time
        x, y, heading = self.pose
        dist = speed * dt

        # advance robot by given distance and angle
        if abs(joint_angle) < 0.0000001:  # EPS
            # Straight movement - a special case
            x += dist * math.cos(heading)
            y += dist * math.sin(heading)
            # Not needed: heading += angle
        else:
            # Arc
            radius = (FRONT_REAR_AXIS_DISTANCE/2) / math.tan(joint_angle)
            angle = dist / radius
            x += dist * math.cos(heading)
            y += dist * math.sin(heading)
            heading += angle  # not normalized
        self.pose = (x, y, heading)
        self.publish('pose2d', [round(x*1000), round(y*1000), round(math.degrees(heading)*100)])

    def send_esp(self, data):
        self.counter += 1
        crc = 0xFF & (256 - (sum(data) + len(data) + 1 + self.counter))
        self.publish('esp_data', bytes([SYNC]) + add_esc_chars(bytes([len(data) + 1, self.counter & 0xFF]) + data + bytes([crc])))

    def send_speed(self):
        data = b'G' + struct.pack('<hh', int(self.desired_speed * 1000), int(self.desired_steering_angle_deg * 100))
        return self.send_esp(data)

    def on_tick(self, data):
        if self.odometry_requested:
            self.send_speed()
        else:
            self.odometry_requested = True
            self.send_esp(b'T'+ struct.pack('<HH', 100, 150))  # keep watchdog short

    def on_esp_data(self, data):
        self.buf += remove_esc_chars(data)
        if SYNC not in self.buf:
            return
        self.buf = self.buf[self.buf.index(SYNC):]
        if len(self.buf) < 4:
            return  # SYNC, len, cmd, crc
        length = self.buf[1]
        if len(self.buf) < length + 3:  # SYNC, len and CRC are not counted
            return  # message not completed yet - timeout??
        crc = sum(self.buf[1:length + 3])
        if crc & 0xFF != 0:
            logging.warning(f'CRC error {(crc, self.buf, self.buf.hex())}')
            return
        packet = self.buf[2:length + 2]
        self.buf = self.buf[length + 3:]
        if len(packet) == 2:
            # ACK/NAACK
            if packet[1] != ord('A'):  # packet[0] != self.counter ... ignored for now
                logging.warning(f'Unexpected message: {(packet, packet.hex(), self.counter)}')
        elif len(packet) == 20:
            assert packet[1] == ord('I'), packet[1]
            speed, joint_angle = self.parse_odometry(packet)
            self.publish_pose2d(speed, joint_angle)
        else:
            assert 0, (length, packet.hex())

    def on_desired_steering(self, data):
        """
        Store desired speed and steering angle.
        Input is the pair of speed in millimeters per second and steering angle in hundredth of degree
        """
        speed_mm_per_sec, steering_deg_hundredth = data
        self.desired_speed = speed_mm_per_sec/1000  # m/s
        if self.max_speed is not None:
            self.desired_speed = min(self.max_speed, max(-self.max_speed, self.desired_speed))
        self.desired_steering_angle_deg = steering_deg_hundredth/100  # degrees
        if self.max_steering_deg is not None:
            self.desired_steering_angle_deg = min(self.max_steering_deg,
                                                  max(-self.max_steering_deg, self.desired_steering_angle_deg))

    def draw(self):
        import matplotlib.pyplot as plt

        for selection in range(4):
            t = [a[0] for a in self.debug_arr]
            x = [a[1][selection] for a in self.debug_arr]
            line = plt.plot(t, x, '-o', linewidth=2, label=f'enc {selection + 1}')

        plt.xlabel('time (s)')
        plt.ylabel('enc')
        plt.legend()
        plt.show()
