"""
  Spider3 Rider Driver
"""


import struct
from threading import Thread

from osgar.bus import BusShutdownException


def CAN_triplet(msg_id, data):
    return [msg_id, bytes(data), 0]  # flags=0, i.e basic addressing


class Spider(Thread):
    def __init__(self, config, bus):
        bus.register('can', 'status')
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus
        self.status_word = None  # not defined yet
        self.wheel_angles = None  # four wheel angles as received via CAN
        self.zero_steering = None  # zero position of all 4 wheels
        self.speed_cmd = [0, 0]
        self.status_cmd = 3
        self.alive = 0  # toggle with 128
        self.desired_angle = None  # in Spider mode desired weels direction
        self.desired_speed = None
        self.verbose = False  # TODO node

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
                self.status_word = struct.unpack('H', packet[2:])[0]
                if self.wheel_angles is not None and self.zero_steering is not None:
                    ret = [self.status_word, [Spider.fix_range(a - b) for a, b in zip(self.wheel_angles, self.zero_steering)]]
                else:
                    ret = [self.status_word, None]

                # handle steering
                if self.desired_angle is not None and self.desired_speed is not None:
                    self.send((self.desired_speed, self.desired_angle))
                else:
                    self.send((0, 0))
                return ret

            elif msg_id == 0x201:
                assert len(packet) == 2 + 8, packet
                self.wheel_angles = struct.unpack_from('HHHH', packet, 2)
                if verbose and self.wheel_angles is not None and self.zero_steering is not None:
                    print('Wheels:',
                          [Spider.fix_range(a - b) for a, b in zip(self.wheel_angles, self.zero_steering)])
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
                if verbose:
                    print("Enc:", val)

    def process_gen(self, data, verbose=False):
        self.buf, packet = self.split_buffer(self.buf + data)
        while len(packet) > 0:
            ret = self.process_packet(packet, verbose=verbose)
            if ret is not None:
                yield ret
            self.buf, packet = self.split_buffer(self.buf)  # i.e. process only existing buffer now

    def run(self):
        try:
            while True:
                dt, channel, data = self.bus.listen()
                if channel == 'can':
                    status = self.process_packet(data, verbose=self.verbose)
                    if status is not None:
                        self.bus.publish('status', status)
                elif channel == 'move':
                    self.desired_speed, self.desired_angle = data
                else:
                    assert False, channel  # unsupported channel
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

    def send(self, data):
        if True:  #self.can_bridge_initialized:
            speed, angular_speed = data
            if speed > 0:
                if self.status_word is None or self.status_word & 0x10 != 0:
                    angle_cmd = int(angular_speed)  # TODO verify angle, byte resolution
                else:
                    print('SPIDER MODE')
                    desired_angle = int(angular_speed)  # TODO proper naming etc.
                    if self.wheel_angles is not None and self.zero_steering is not None:
                        curr = Spider.fix_range(self.wheel_angles[0] - self.zero_steering[0])
                        diff = Spider.fix_range(desired_angle - curr)
                        print('DIFF', diff)
                        if abs(diff) < 5:
                            angle_cmd = 0
                        elif diff < 0:
                            angle_cmd = 50
                        else:
                            angle_cmd = 0x80 + 50
                    else:
                        angle_cmd = 0
                if speed >= 10:
                    packet = CAN_triplet(0x401, [0x80 + 127, angle_cmd])
                else:
                    packet = CAN_triplet(0x401, [0x80 + 80, angle_cmd])
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

# vim: expandtab sw=4 ts=4
