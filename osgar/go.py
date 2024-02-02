"""
  Test basic driving functionality
"""
import math
from datetime import timedelta

from osgar.node import Node


class Go(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed')
        self.start_pose = None
        self.traveled_dist = 0.0
        self.verbose = False
        self.speed = config['max_speed']
        self.dist = config['dist']
        self.timeout = timedelta(seconds=config['timeout'])
        self.desired_spider_angle = config.get('desired_angle', 0.0)
        self.repeat = config.get('repeat', 1)
        self.emergency_stop = None

    def send_speed_cmd(self, speed, angular_speed):
        return self.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])

    def on_pose2d(self, data):
        x, y, heading = data
        pose = (x / 1000.0, y / 1000.0, math.radians(heading / 100.0))
        if self.start_pose is None:
            self.start_pose = pose
        self.traveled_dist = math.hypot(pose[0] - self.start_pose[0], pose[1] - self.start_pose[1])

    def on_emergency_stop(self, data):
        self.emergency_stop = data

    def wait(self, dt):  # TODO refactor to some common class
        if self.time is None:
            self.update()
        start_time = self.time
        while self.time - start_time < dt:
            self.update()

    def sub_run(self):
        self.update()  # define self.time
        print(self.time, "Go!")
        start_time = self.time
        if self.dist >= 0:
            self.send_speed_cmd(self.speed, self.desired_spider_angle)
        else:
            self.send_speed_cmd(-self.speed, self.desired_spider_angle)
        while self.traveled_dist < abs(self.dist) and self.time - start_time < self.timeout:
            self.update()
            if self.emergency_stop:
                print(self.time, "(sub_run) Emergency STOP")
                break
        print(self.time, "STOP")
        self.send_speed_cmd(0.0, 0.0)
        self.wait(timedelta(seconds=1))
        print(self.time, "distance:", self.traveled_dist, "time:", (self.time - start_time).total_seconds())

    def run(self):
        for run_number in range(self.repeat):
            self.traveled_dist = 0.0
            self.start_pose = None
            self.sub_run()
            if self.emergency_stop:
                print(self.time, "(run) Emergency STOP")
                break
            self.dist = -self.dist  # next run will be in opposite direction


if __name__ == "__main__":
    import argparse
    import os.path
    from osgar.replay import replay
    from osgar.record import record
    from osgar.lib.config import config_load

    parser = argparse.ArgumentParser(description='go')
    subparsers = parser.add_subparsers(help='sub-command help', dest='command')
    subparsers.required = True
    parser_run = subparsers.add_parser('run', help='run on real HW')
    parser_run.add_argument('config', nargs='+', help='configuration file')
    parser_run.add_argument('--note', help='add description')
    parser_run.add_argument('--speed', '-s', help="speed in m/s (default: from config)", type=float)
    parser_run.add_argument('--dist', '-d', help="distance in m (default: %(default)sm)", type=float, default=1.0)
    parser_run.add_argument('--timeout', help='seconds before stopping (default: %(default)s)', type=int, default=10)
    parser_run.add_argument('--spider-angle', '-a', help='for Spider mode desired angle in degrees', type=int, default=0)

    parser_replay = subparsers.add_parser('replay', help='replay from logfile')
    parser_replay.add_argument('logfile', help='recorded log file')
    parser_replay.add_argument('--force', '-F', dest='force', action='store_true', help='force replay even for failing output asserts')
    parser_replay.add_argument('--config', nargs='+', help='force alternative configuration file')
    parser_replay.add_argument('--verbose', '-v', help="verbose mode", action='store_true')

    args = parser.parse_args()
    print(args)

    if args.command == 'replay':
        args.module = 'app'
        game = replay(args, application=Go)
        game.verbose = args.verbose
        game.run()

    elif args.command == 'run':
        prefix = 'go-' + os.path.basename(args.config[0]).split('.')[0] + '-'
        cfg = config_load(*args.config, application=Go)

        # apply overrides from command line
        cfg['robot']['modules']['app']['init']['dist'] = args.dist
        cfg['robot']['modules']['app']['init']['timeout'] = args.timeout
        if args.speed is not None:
            cfg['robot']['modules']['app']['init']['max_speed'] = args.speed
        if args.spider_angle is not None:
            cfg['robot']['modules']['app']['init']['desired_angle'] = math.radians(args.spider_angle)

        record(cfg, prefix)

# vim: expandtab sw=4 ts=4
