"""
  Follow Me (primary target Eduro robot)
"""
import sys
import math
from datetime import timedelta

# Notes:
# Expects SICK laser scan with 270 degrees field of view mounted in front.


# TODO shared place for multiple applications
class EmergencyStopException(Exception):
    pass


def min_dist(laser_data):
    if len(laser_data) > 0:
        # remove ultra near reflections and unlimited values == 0
        laser_data = [x if x > 10 else 10000 for x in laser_data]
        return min(laser_data)
    return 0


class FollowMe:
    def __init__(self, config, bus):
        self.bus = bus
        self.last_position = [0, 0, 0]  # proper should be None, but we really start from zero
        self.time = None
        self.raise_exception_on_stop = False
        self.verbose = False
        self.last_scan = None

    def update(self):
        packet = self.bus.listen()
        if packet is not None:
            timestamp, channel, data = packet
            self.time = timestamp
            if channel == 'pose2d':
                self.last_position = data
            elif channel == 'scan':
                if self.verbose:
                    print(min_dist(data)/1000.0)
                self.last_scan = data
            elif channel == 'emergency_stop':
                if self.raise_exception_on_stop and data:
                    raise EmergencyStopException()

    def wait(self, dt):  # TODO refactor to some common class
        if self.time is None:
            self.update()
        start_time = self.time
        while self.time - start_time < dt:
            self.update()

    def send_speed_cmd(self, speed, angular_speed):
        return self.bus.publish('desired_speed', 
                [round(speed*1000), round(math.degrees(angular_speed)*100)])

    def followme(self):
        print("Follow Me!")

        SCAN_SIZE = 811  # TODO config
        SCANS_PER_DEG = SCAN_SIZE//270

        # limit tracking to front 180deg only due to mounting (back laser is blocked by robot body)
        LIMIT_LOW = 0  # SCAN_SIZE//6
        LIMIT_HIGH = SCAN_SIZE  # 5*SCAN_SIZE//6
        CLOSE_REFLECTIONS = 10  # ignore readings closer than 10mm, where 0 = infinite (no response)

        thresholds = []
        for i in range(SCAN_SIZE):
            if LIMIT_LOW <= i <= LIMIT_HIGH:
                deg = -135 + 270 * i / SCAN_SIZE
                rad = math.radians(deg)
                thresh = 1000 * (0.17 + 0.17 * max(0, math.cos(rad)))  # [mm]
            else:
                thresh = 0
            thresholds.append(thresh)

        masterAngleOffset = 0  # TODO
        index = None
        while True:
            if self.last_scan is not None:
                assert len(self.last_scan) == SCAN_SIZE, len(self.last_scan)
                near = 10.0
                if index is None:
                    low = LIMIT_LOW
                    high = LIMIT_HIGH
                else:
                    # search in +/- 10deg sector
                    low = max(LIMIT_LOW, index - 10 * SCANS_PER_DEG)
                    high = min(LIMIT_HIGH, index + 10 * SCANS_PER_DEG)

                for i in range(low, high):
                    x = self.last_scan[i]
                    if CLOSE_REFLECTIONS < x < near * 1000:
                        near = x / 1000.0
                        index = i
                maxnear = min( (x for x in self.last_scan if x > CLOSE_REFLECTIONS) ) / 1000.0

                if self.verbose:
                    print(near, maxnear, index)

                if near > 1.3 or any(x < thresh for (x, thresh) in zip(self.last_scan, thresholds) if x > CLOSE_REFLECTIONS):
                    self.send_speed_cmd(0.0, 0.0)
                else:
                    angle = math.radians(270 * index/SCAN_SIZE - 135) + masterAngleOffset
                    desiredAngle = 0
                    desiredDistance = 0.4
                    speed = 0.2 + 2 * (near - desiredDistance)
                    rot = 1.5 * (angle - desiredAngle)
                    if speed < 0:
                        speed = 0
                    self.send_speed_cmd(speed, rot)

                self.last_scan = None
            self.update()

    def run(self):
        try:
            self.raise_exception_on_stop = True
            self.followme()
        except EmergencyStopException:
            print('!!!Emergency STOP!!!')
            self.raise_exception_on_stop = False
            self.send_speed_cmd(0.0, 0.0)
            self.wait(timedelta(seconds=1))

    def start(self):
        pass

    def request_stop(self):
        self.bus.shutdown()

    def join(self):
        pass


if __name__ == "__main__":
    from osgar.lib.config import load as config_load
    from osgar.record import Recorder
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
    parser_replay.add_argument('--verbose', '-v', help="verbose mode", action='store_true')
    args = parser.parse_args()

    if args.command == 'replay':
        from replay import replay
        args.module = 'app'
        game = replay(args, application=FollowMe)
        game.verbose = args.verbose
        game.run()

    elif args.command == 'run':
        log = LogWriter(prefix='followme-', note=str(sys.argv))
        config = config_load(*args.config)
        log.write(0, bytes(str(config), 'ascii'))  # write configuration
        robot = Recorder(config=config['robot'], logger=log, application=FollowMe)
        game = robot.modules['app']  # TODO nicer reference
        robot.start()
        game.run()
        robot.finish()

# vim: expandtab sw=4 ts=4
