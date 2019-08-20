"""
  Test basic driving functionality
"""
import math
from datetime import timedelta

from osgar.node import Node


class Go(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.start_pose = None
        self.traveled_dist = 0.0
        self.verbose = False
        self.speed = config['max_speed']
        self.dist = config['dist']
        self.timeout = timedelta(seconds=config['timeout'])

    def send_speed_cmd(self, speed, angular_speed):
        return self.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])

    def update(self):
        channel = super().update()  # define self.time
        if self.verbose:
            print(self.time, 'Go', channel)
        if channel == 'pose2d':
            x, y, heading = self.pose2d
            pose = (x/1000.0, y/1000.0, math.radians(heading/100.0))
            if self.start_pose is None:
                self.start_pose = pose
            self.traveled_dist = math.hypot(pose[0] - self.start_pose[0], pose[1] - self.start_pose[1])

    def wait(self, dt):  # TODO refactor to some common class
        if self.time is None:
            self.update()
        start_time = self.time
        while self.time - start_time < dt:
            self.update()

    def run(self):
        print(self.time, "Go One Meter!")
        self.update()  # define self.time
        start_time = self.time
        if self.dist >= 0:
            self.send_speed_cmd(self.speed, 0.0)
        else:
            self.send_speed_cmd(-self.speed, 0.0)
        while self.traveled_dist < abs(self.dist) and self.time - start_time < self.timeout:
            self.update()
        print(self.time, "STOP")
        self.send_speed_cmd(0.0, 0.0)
        self.wait(timedelta(seconds=1))
        print(self.time, "distance:", self.traveled_dist, "time:", (self.time - start_time).total_seconds())


if __name__ == "__main__":
    import argparse
    import os.path
    import sys
    from osgar.replay import replay
    from osgar.lib.config import load as config_load
    from osgar.record import Recorder
    from osgar.logger import LogWriter

    parser = argparse.ArgumentParser(description='go')
    subparsers = parser.add_subparsers(help='sub-command help', dest='command')
    subparsers.required = True
    parser_run = subparsers.add_parser('run', help='run on real HW')
    parser_run.add_argument('config', nargs='+', help='configuration file')
    parser_run.add_argument('--note', help='add description')
    parser_run.add_argument('--speed', '-s', help="speed in m/s (default: from config)", type=float)
    parser_run.add_argument('--dist', '-d', help="distance in m (default: %(default)sm)", type=float, default=1.0)
    parser_run.add_argument('--timeout', help='seconds before stopping (default: %(default)s)', type=int, default=10)

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
        log = LogWriter(prefix=prefix, note=str(sys.argv))
        config = config_load(*args.config)

        # apply overrides from command line
        config['robot']['modules']['app']['init']['dist'] = args.dist
        config['robot']['modules']['app']['init']['timeout'] = args.timeout
        if args.speed is not None:
            config['robot']['modules']['app']['init']['max_speed'] = args.speed

        log.write(0, bytes(str(config), 'ascii'))  # write configuration
        robot = Recorder(config=config['robot'], logger=log, application=Go)
        game = robot.modules['app']
        robot.start()
        game.run()
        robot.finish()

# vim: expandtab sw=4 ts=4
