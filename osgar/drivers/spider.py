"""
  Spider3 Rider Driver
"""

import struct
import ctypes
import math

from osgar.node import Node
from osgar.bus import BusShutdownException


SPIDER_ENC_SCALE = 0.0023
WHEEL_DISTANCE = 1.5  # TODO calibrate


def CAN_triplet(msg_id, data):
    return [msg_id, bytes(data), 0]  # flags=0, i.e basic addressing


def sint8_diff(a, b):
    return ctypes.c_int8(a - b).value


class Spider(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('can', 'status', 'encoders', 'pose2d')

        self.bus = bus
        self.status_word = None  # not defined yet
        self.wheel_angles = None  # four wheel angles as received via CAN
        self.zero_steering = None  # zero position of all 4 wheels
        self.speed_cmd = [0, 0]
        self.status_cmd = 3
        self.alive = 0  # toggle with 128
        self.desired_angle = None  # in Spider mode desired weels direction
        self.desired_speed = None
        self.speed_history_left = []
        self.speed_history_right = []
        self.speed = None  # unknown
        self.verbose = False  # TODO node
        self.debug_arr = []
        self.pose2d = (0.0, 0.0, 0.0)  # x, y in meters, heading in radians (not corrected to 2PI)
        self.prev_enc = None  # not defined
        self.paused = True  # safe default
        self.valve = None

    def update_speed(self, diff):
        self.speed_history_left.append(diff[0])
        self.speed_history_right.append(diff[1])
        self.speed_history_left = self.speed_history_left[-10:]
        self.speed_history_right = self.speed_history_right[-10:]

        self.speed = SPIDER_ENC_SCALE * (sum(self.speed_history_left) + sum(self.speed_history_right))  # 20Hz -> left + right = 1s

    def update_pose2d(self, diff):
        x, y, heading = self.pose2d

        metricL = SPIDER_ENC_SCALE * diff[0]
        metricR = SPIDER_ENC_SCALE * diff[1]

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
        self.pose2d = (x, y, heading)

    def send_pose2d(self):
        x, y, heading = self.pose2d
        self.bus.publish('pose2d', [round(x*1000), round(y*1000), round(math.degrees(heading)*100)])

    @staticmethod
    def fix_range(value):
        "into <-256, +256) interval"
        if value < -256:
            value += 512
        elif value >= 256:
            value -= 512
        return value

    def process_packet(self, data, verbose=False):
        msg_id, packet, flags = data
        if True:  #len(packet) >= 2:
#            msg_id = ((packet[0]) << 3) | (((packet[1]) >> 5) & 0x1f)
            packet = b'XX' + packet  # hack for backward compatibility
            if verbose:
                print(hex(msg_id), packet[2:])
            if msg_id == 0x200:
                prev = self.status_word
                self.status_word = struct.unpack('H', packet[2:])[0]
                if prev is None or (prev & 0x7FFF != self.status_word & 0x7FFF):
                    self.paused = (self.status_word & 0x10) == 0
                    mode = 'CAR' if self.status_word & 0x10 else 'SPIDER'
                    print(self.time, hex(self.status_word & 0x7FFF), 'Mode:', mode)
                if self.wheel_angles is not None and self.zero_steering is not None:
                    ret = [self.status_word, [Spider.fix_range(a - b) for a, b in zip(self.wheel_angles, self.zero_steering)]]
                else:
                    ret = [self.status_word, None]

                # handle steering
                if self.desired_angle is not None and self.desired_speed is not None and not self.paused:
                    self.send_speed((self.desired_speed, self.desired_angle))
                else:
                    self.send_speed((0, 0))
                return ret

            elif msg_id == 0x201:
                assert len(packet) == 2 + 8, packet
                self.wheel_angles = struct.unpack_from('HHHH', packet, 2)
                if verbose and self.wheel_angles is not None and self.zero_steering is not None:
                    print('Wheels:',
                          [Spider.fix_range(a - b) for a, b in zip(self.wheel_angles, self.zero_steering)])
            elif msg_id == 0x202:
                assert len(packet) == 2 + 8, packet
                valves = struct.unpack_from('HHHH', packet, 2)
                self.valve = (valves[0] - valves[2], valves[1] - valves[3])
            elif msg_id == 0x203:
                assert len(packet) == 2 + 8, packet
                prev = self.zero_steering
                self.zero_steering = struct.unpack_from('HHHH', packet, 2)
                if verbose:
                    print('Zero', self.zero_steering)
                # make sure that calibration did not change during program run
                assert prev is None or prev == self.zero_steering, (prev, self.zero_steering)
            elif msg_id == 0x204:
                assert len(packet) == 2 + 8, packet
                val = struct.unpack_from('HHBBH', packet, 2)
                if verbose:
                    print("User:", val[2]&0x7F, val[3]&0x7F, val)
            elif msg_id == 0x2A0:
                # encoders
                assert len(packet) == 2 + 4, packet
                val = struct.unpack_from('hh', packet, 2)
                if self.prev_enc is None:
                    self.prev_enc = val
                diff = [sint8_diff(a, b) for a, b in zip(val, self.prev_enc)]
                self.publish('encoders', list(diff))
                self.update_speed(diff)
                self.update_pose2d(diff)
                self.send_pose2d()
                self.prev_enc = val
                if verbose:
                    print("Enc:", val)
                    if self.valve is not None:
                        self.debug_arr.append([self.time.total_seconds(), *diff] + list(self.valve) + [self.speed])

    def process_gen(self, data, verbose=False):
        self.buf, packet = self.split_buffer(self.buf + data)
        while len(packet) > 0:
            ret = self.process_packet(packet, verbose=verbose)
            if ret is not None:
                yield ret
            self.buf, packet = self.split_buffer(self.buf)  # i.e. process only existing buffer now


    def update(self):
        channel = super().update()
        if channel == 'can':
            status = self.process_packet(self.can, verbose=self.verbose)
            if status is not None:
                self.bus.publish('status', status)
        elif channel == 'move':
            self.desired_speed, self.desired_angle = self.move
        else:
            assert False, channel  # unsupported channel

    def send_speed(self, data):
        if True:  #self.can_bridge_initialized:
            speed, angular_speed = data
            if abs(speed) > 0:
#                print(self.status_word & 0x7fff)
                if self.status_word is None or self.status_word & 0x10 != 0:
                    # car mode
                    angle_cmd = int(angular_speed)  # TODO verify angle, byte resolution
                else:
                    # spider mode
                    desired_angle = int(angular_speed)  # TODO proper naming etc.
                    if self.wheel_angles is not None and self.zero_steering is not None:
                        curr = Spider.fix_range(self.wheel_angles[0] - self.zero_steering[0])
                        diff = Spider.fix_range(desired_angle - curr)
#                        print('DIFF', diff)
                        if abs(diff) < 5:
                            angle_cmd = 0
                        elif diff < 0:
                            angle_cmd = 50
                        else:
                            angle_cmd = 0x80 + 50
                    else:
                        angle_cmd = 0
                sign_offset = 0x80 if speed > 0 else 0x0
                if abs(speed) >= 10:
                    packet = CAN_triplet(0x401, [sign_offset + 127, angle_cmd])
                else:
                    packet = CAN_triplet(0x401, [sign_offset + 80, angle_cmd])
            else:
                packet = CAN_triplet(0x401, [0, 0])  # STOP packet
            self.bus.publish('can', packet)

            # alive
            packet = CAN_triplet(0x400, [self.status_cmd, self.alive])
            self.bus.publish('can', packet)
            self.alive = 128 - self.alive
        else:
            print('CAN bridge not initialized yet!')
#            self.logger.write(0, 'ERROR: CAN bridge not initialized yet! [%s]' % str(data))

    def draw(self):
        # for debugging
        import matplotlib.pyplot as plt
        arr = self.debug_arr
        t = [a[0] for a in arr]
        values = [a[1:] for a in arr]

        line = plt.plot(t, values, '-o', linewidth=2)

        plt.xlabel('time (s)')
        plt.legend(['left', 'right'])
        plt.show()


# vim: expandtab sw=4 ts=4
