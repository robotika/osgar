"""
  Boat Marina 2.0 - experiments with osgar.logger and robot container
"""

import argparse
import sys
import time
from datetime import timedelta

from osgar.logger import LogWriter, LogReader
from osgar.lib.config import load as config_load
from osgar.robot import Robot

from osgar.bus import BusHandler


class BoatMarina2:

    def __init__(self, config, bus):
        self.bus = bus
        self.time = None

    def update(self):
        packet = self.bus.listen()
        if packet is not None:
            timestamp, channel, data = packet
            self.time = timestamp

    def start(self):
        pass

    def play(self):
        #self.bus.publish('raw', b'$ARRST\r\n')
#        self.bus.publish('raw', [0x1E, 'R', 0x10, 1])
        self.bus.publish('move', [999, 998])  # uniq values for test
        for i in range(10):
            time.sleep(1)  # TODO use self.time/wait()
            #self.bus.publish('raw', b'$CHV,1000,1001,1000,1000*5C\r\n')
            #self.bus.publish('raw', [0x1E, 'R', 0x03, 6])
            self.bus.publish('move', [600, 1000])
        self.bus.publish('move', [1000, 1000])

    def request_stop(self):
        self.bus.shutdown()

    def join(self):
        pass

    def register(self, callback):
        self.monitors.append(callback)
        return callback

    def unregister(self, callback):
        assert callback in self.monitors
        self.monitors.remove(callback)

    def wait(self, dt):
        if self.time is None:
            self.update()
        start_time = self.time
        while self.time - start_time < dt:
            self.update()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Boat Marina 2.0')
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
        game = replay(args, application=BoatMarina2)
        game.play()

    elif args.command == 'run':
        log = LogWriter(prefix='boat-', note=str(sys.argv))
        config = config_load(*args.config)
        log.write(0, bytes(str(config), 'ascii'))  # write configuration
        robot = Robot(config=config['robot'], logger=log, application=BoatMarina2)
        game = robot.modules['app']  # TODO nicer reference
        robot.start()
        game.play()
        robot.finish()
    else:
        assert False, args.command  # unsupported command

# vim: expandtab sw=4 ts=4
