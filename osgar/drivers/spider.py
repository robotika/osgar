"""
  Spider3 Rider Driver
"""

import sys
import os
import inspect

OSGAR_ROOT = os.path.realpath(os.path.abspath(os.path.join(os.path.split(inspect.getfile( inspect.currentframe() ))[0],"..")))
if OSGAR_ROOT not in sys.path:
    sys.path.insert(0, OSGAR_ROOT) # access to logger without installation


import serial
import struct
from threading import Thread

from lib.logger import LogWriter, LogReader
from drivers.bus import BusShutdownException


CAN_BRIDGE_READY = b'\xfe\x10'  # CAN bridge is ready to accept configuration commands
CAN_BRIDGE_SYNC = b'\xFF'*10    # CAN bridge synchronization bytes
CAN_SPEED_1MB = b'\xfe\x57'     # configure CAN bridge to communicate on 1Mb CAN network
CAN_BRIDGE_START = b'\xfe\x31'  # start bridge


def CAN_packet(msg_id, data):
    header = [(msg_id>>3) & 0xff, (msg_id<<5) & 0xe0 | (len(data) & 0xf)]
    return bytes(header + data)


class Spider(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus
        self.buf = b''

        self.can_bridge_initialized = False
        self.status_word = None  # not defined yet
        self.wheel_angles = None  # four wheel angles as received via CAN
        self.zero_steering = None  # zero position of all 4 wheels
        self.speed_cmd = [0, 0]
        self.status_cmd = 3
        self.alive = 0  # toggle with 128
        self.desired_angle = None  # in Spider mode desired weels direction
        self.desired_speed = None

    @staticmethod
    def split_buffer(data):
        # skip 0xFF prefix bytes (CAN bridge control bytes)
        data = data.lstrip(b'\xff')

        if len(data) >= 2:
            # see https://en.wikipedia.org/wiki/CAN_bus
            header = data[:2]
            rtr = (header[1] >> 4) & 0x1  # Remote transmission request
            size = (header[1]) & 0x0f
            if rtr:
                return data[2:], header
            elif len(data) >= 2 + size:
                return data[2+size:], data[:2+size]
        return data, b''  # no complete packet available yet

    @staticmethod
    def fix_range(value):
        "into <-256, +256) interval"
        if value < -256:
            value += 512
        elif value >= 256:
            value -= 512
        return value

    def process_packet(self, packet, verbose=False):
        if packet == CAN_BRIDGE_READY:
            self.bus.publish('can', CAN_BRIDGE_SYNC)
            self.bus.publish('can', CAN_SPEED_1MB)
            self.bus.publish('can', CAN_BRIDGE_START)
            self.can_bridge_initialized = True
            return None

        if len(packet) >= 2:
            msg_id = ((packet[0]) << 3) | (((packet[1]) >> 5) & 0x1f)
            if verbose:
                print(hex(msg_id), packet[2:])
            if msg_id == 0x200:
                self.status_word = struct.unpack('H', packet[2:])[0]
                if self.wheel_angles is not None and self.zero_steering is not None:
                    ret = self.status_word, tuple([Spider.fix_range(a - b) for a, b in zip(self.wheel_angles, self.zero_steering)])
                else:
                    ret = self.status_word, None

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
                if channel == 'raw':
                    if len(data) > 0:
                        for status in self.process_gen(data):
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
        if self.can_bridge_initialized:
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
                    packet = CAN_packet(0x401, [0x80 + 127, angle_cmd])
                else:
                    packet = CAN_packet(0x401, [0x80 + 80, angle_cmd])
            else:
                packet = CAN_packet(0x401, [0, 0])  # STOP packet
            self.bus.publish('can', packet)

            # alive
            packet = CAN_packet(0x400, [self.status_cmd, self.alive])
            self.bus.publish('can', packet)
            self.alive = 128 - self.alive
        else:
            print('CAN bridge not initialized yet!')
#            self.logger.write(0, 'ERROR: CAN bridge not initialized yet! [%s]' % str(data))


if __name__ == "__main__":
    import argparse
    import time
    from lib.config import Config
    from datetime import timedelta
    from drivers.logserial import LogSerial, LogSerialOut
    from drivers.bus import BusHandler
    
    parser = argparse.ArgumentParser(description='Test robot configuration')
    subparsers = parser.add_subparsers(help='sub-command help', dest='command')
    subparsers.required = True
    parser_run = subparsers.add_parser('run', help='run on real HW')
    parser_run.add_argument('config', help='configuration file')
    parser_run.add_argument('--note', help='add description')
    parser_run.add_argument('--turn', help='test desired steering angle', type=int, default=50)

    parser_replay = subparsers.add_parser('replay', help='replay from logfile')
    parser_replay.add_argument('logfile', help='recorded log file')
    parser_replay.add_argument('--stream', help='stream ID', type=int, default=None)
    parser_replay.add_argument('--times', help='display timestamps', action='store_true')
    parser_replay.add_argument('--verbose', '-v', dest='verbose', action='store_true',
                               help='verbose output')
    args = parser.parse_args()

    if args.command == 'run':
        log = LogWriter(prefix='spider-', note=str(sys.argv))
        config = Config.load(args.config)
        log.write(0, bytes(str(config.data), 'ascii'))  # write configuration
        config_serial = config.data['robot']['spider']
        device_out = LogSerialOut(config_serial, bus=BusHandler(log, out={}))
        spider = Spider(config=config.data['robot']['spider'], bus=BusHandler(log, out={2:[(device_out.bus.queue, 2)]}))
        device = LogSerial(config_serial, bus=BusHandler(log, out={1:[(spider.bus.queue, 1)]}), com=device_out.com)
        # TODO beware of ignored parameters rtscts=True, dsrdtr=True!
        device.com.setRTS()  # move this to some CANSerial? or part of config
        device.com.setDTR(0)
        spider.start()
        device.start()
        device_out.start()

        spider.desired_angle = args.turn
        time.sleep(15.0)
        spider.desired_angle = None
        time.sleep(5.0)

        spider.request_stop()
        device.request_stop()
        device_out.request_stop()

        spider.join()
        device.join()
        device_out.join()

    elif args.command == 'replay':
        class DummyBus:
            def publish(self, stream_id, data):
                pass
        # TODO read streams directly from log file configuration
        spider = Spider(config={'stream_id_in':1, 'stream_id_out':2}, bus=DummyBus())
        with LogReader(args.logfile) as log:
            for timestamp, stream_id, data in log.read_gen(args.stream):
                if args.times:
                    print(timestamp)
                for ret in spider.process_gen(data, verbose=args.verbose):
                    if ret is not None :
                        print(hex(ret[0]), ret[1])

# vim: expandtab sw=4 ts=4
