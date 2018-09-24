"""
  Follow Me (primary target Eduro robot)
"""
import sys
import math
from datetime import timedelta

from osgar.lib.config import load as config_load
from osgar.robot import Robot


wd = 427.0 / 445.0 * 0.26/4.0 # with gear  1:4
ENC_SCALE = math.pi * wd / 0x10000


def min_dist(laser_data):
    if len(laser_data) > 0:
        # remove ultra near reflections and unlimited values == 0
        laser_data = [x if x > 10 else 10000 for x in laser_data]
        return min(laser_data)
    return 0


class FollowMe:
    def __init__(self, config, bus):
        self.bus = bus
        self.last_position = None
        self.traveled_dist = 0.0  # in meters
        self.time = None

    def update(self):
        packet = self.bus.listen()
        if packet is not None:
            timestamp, channel, data = packet
            self.time = timestamp
            if channel == 'encoders':
                self.traveled_dist = ENC_SCALE*(data[0] + data[1])/2
                print('Dist: %.2f' % self.traveled_dist)
            elif channel == 'scan':
                print(min_dist(data)/1000.0)

    def wait(self, dt):  # TODO refactor to some common class
        if self.time is None:
            self.update()
        start_time = self.time
        while self.time - start_time < dt:
            self.update()

    def play(self):
        print("FollowMe!")
        self.bus.publish('desired_speed', [0.2, 0.0])
        while self.traveled_dist < 1.0:
            self.update()
        self.bus.publish('desired_speed', [0.0, 0.0])
        self.wait(timedelta(seconds=3))

    def start(self):
        pass

    def request_stop(self):
        self.bus.shutdown()

    def join(self):
        pass


if __name__ == "__main__":
    from osgar.logger import LogWriter, LogReader
    import argparse

    parser = argparse.ArgumentParser(description='Follow Me')
    subparsers = parser.add_subparsers(help='sub-command help', dest='command')
    subparsers.required = True
    parser_run = subparsers.add_parser('run', help='run on real HW')
    parser_run.add_argument('config', nargs='+', help='configuration file')
    parser_run.add_argument('--note', help='add description')

    parser_replay = subparsers.add_parser('replay', help='replay from logfile')
    parser_replay.add_argument('logfile', help='recorded log file')
    parser_replay.add_argument('--force', '-F', dest='force', action='store_true', help='force replay even for failing output asserts')
    parser_replay.add_argument('--config', nargs='+', help='force alternative configuration file')
    args = parser.parse_args()

    if args.command == 'replay':
        from replay import replay
        args.module = 'app'
        game = replay(args, application=FollowMe)
        game.play()

    elif args.command == 'run':
        log = LogWriter(prefix='eduro-', note=str(sys.argv))
        config = config_load(*args.config)
        log.write(0, bytes(str(config), 'ascii'))  # write configuration
        robot = Robot(config=config['robot'], logger=log, application=FollowMe)
        game = robot.modules['app']  # TODO nicer reference
        robot.start()
        game.play()
        robot.finish()

# vim: expandtab sw=4 ts=4
