"""
  Test basic driving functionality
"""
import math
from datetime import timedelta

from osgar.node import Node
from osgar.lib.mathex import normalizeAnglePIPI
from osgar.bus import BusShutdownException


class Turn(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.start_pose = None
        self.verbose = False
        self.speed = config['max_speed']
        self.desired_angle = math.radians(config['angle_deg'])
        self.timeout = timedelta(seconds=config['timeout'])
        self.last_position = (0, 0, 0)  # proper should be None, but we really start from zero
        self.max_angular_speed = math.radians(45)
        self.is_moving = None  # unknown

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
            if self.last_position is not None:
                self.is_moving = (self.last_position != pose)

    def wait(self, dt):  # TODO refactor to some common class
        if self.time is None:
            self.update()
        start_time = self.time
        while self.time - start_time < dt:
            self.update()

    def turn(self, angle, with_stop=True, speed=0.0, timeout=None):
        print(self.time, "turn %.1f" % math.degrees(angle))
        start_pose = self.last_position
        if angle >= 0:
            self.send_speed_cmd(speed, self.max_angular_speed)
        else:
            self.send_speed_cmd(speed, -self.max_angular_speed)
        start_time = self.time
        while abs(normalizeAnglePIPI(start_pose[2] - self.last_position[2])) < abs(angle):
            self.update()
            if timeout is not None and self.time - start_time > timeout:
                print(self.time, "turn - TIMEOUT!")
                break
        if with_stop:
            self.send_speed_cmd(0.0, 0.0)
            start_time = self.time
            while self.time - start_time < timedelta(seconds=2):
                self.update()
                if not self.is_moving:
                    break
            print(self.time, 'stop at', self.time - start_time)

    def run(self):
        try:
            self.send_speed_cmd(0, 0)  # trigger processing (workaround for replay)
            self.update()  # define self.time
            print(self.time, "Turn!")
            self.turn(self.desired_angle, timeout=self.timeout)  # TODO radius
        except BusShutdownException:
            pass


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
    parser_run.add_argument('--angle', '-a', help="angle in degrees (default: %(default)sdeg)", type=float, default=90.0)
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
        game = replay(args, application=Turn)
        game.verbose = args.verbose
        game.run()

    elif args.command == 'run':
        prefix = 'turn-' + os.path.basename(args.config[0]).split('.')[0] + '-'
        log = LogWriter(prefix=prefix, note=str(sys.argv))
        config = config_load(*args.config)

        # apply overrides from command line
        config['robot']['modules']['app']['init']['angle_deg'] = args.angle
        config['robot']['modules']['app']['init']['timeout'] = args.timeout
        if args.speed is not None:
            config['robot']['modules']['app']['init']['max_speed'] = args.speed

        log.write(0, bytes(str(config), 'ascii'))  # write configuration
        robot = Recorder(config=config['robot'], logger=log, application=Turn)
        game = robot.modules['app']
        robot.start()
        robot.finish()

# vim: expandtab sw=4 ts=4
