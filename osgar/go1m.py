"""
  Test basic robot/John Deere driving functionality
"""
import sys
from datetime import timedelta

from osgar.lib.config import load as config_load
from osgar.robot import Robot


# meters per single encoder tick
ENC_SCALE = 2*3.39/float(252 + 257)
# TODO move

class GoOneMeter:
    def __init__(self, config, bus):
        self.bus = bus
        self.last_position = None
        self.traveled_dist = 0.0  # in meters
        self.time = None

    def update(self):
        packet = self.bus.listen()
        if packet is not None:
            print('Go1m', packet)
            timestamp, channel, data = packet
            self.time = timestamp
            if channel == 'encoders':
                self.traveled_dist += ENC_SCALE*(data[0] + data[1])/2

    def wait(self, dt):  # TODO refactor to some common class
        if self.time is None:
            self.update()
        start_time = self.time
        while self.time - start_time < dt:
            self.update()

    def play(self):
        print("Go One Meter!")
        self.bus.publish('desired_speed', 0.5)
        while self.traveled_dist < 1.0:
            self.update()
        self.bus.publish('desired_speed', 0.0)
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

    parser = argparse.ArgumentParser(description='Go One Meter')
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
        game = replay(args, application=GoOneMeter)
        game.play()

    elif args.command == 'run':
        log = LogWriter(prefix='go1m-', note=str(sys.argv))
        config = config_load(*args.config)
        log.write(0, bytes(str(config), 'ascii'))  # write configuration
        robot = Robot(config=config['robot'], logger=log, application=GoOneMeter)
        game = robot.modules['app']  # TODO nicer reference
        robot.start()
        game.play()
        robot.finish()

# vim: expandtab sw=4 ts=4
