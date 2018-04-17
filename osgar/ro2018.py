"""
  RoboOrienteering 2018 - experiments with lib.logger and robot container
"""

import argparse
import sys
import math
from datetime import timedelta
from queue import Queue

from osgar.lib.logger import LogWriter, LogReader
from osgar.lib.config import load as config_load
from osgar.drivers import all_drivers
from osgar.robot import Robot

from osgar.drivers.gps import INVALID_COORDINATES
from osgar.drivers.bus import BusHandler


def geo_length(pos1, pos2):
    "return distance on sphere for two integer positions in milliseconds"
    x_scale = math.cos(math.radians(pos1[0]/3600000))
    scale = 40000000/(360*3600000)
    return math.hypot((pos2[0] - pos1[0])*x_scale, pos2[1] - pos1[1]) * scale


def geo_angle(pos1, pos2):
    if geo_length(pos1, pos2) < 1.0:
        return None
    x_scale = math.cos(math.radians(pos1[0]/3600000))
    return math.atan2(pos2[1] - pos1[1], (pos2[0] - pos1[0])*x_scale)


def normalizeAnglePIPI( angle ):
    while angle < -math.pi:
        angle += 2*math.pi
    while angle > math.pi:
        angle -= 2*math.pi
    return angle 


def latlon2xy(lat, lon):
    return int(round(lon*3600000)), int(round(lat*3600000))

class EmergencyStopException(Exception):
    pass

class EmergencyStopMonitor:
    def __init__(self, robot):
        self.robot = robot

    def update(self, robot):
        if (robot.status & RoboOrienteering2018.EMERGENCY_STOP) == 0:
            raise EmergencyStopException()

    # context manager functions
    def __enter__(self):
        self.callback = self.robot.register(self.update)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.robot.unregister(self.callback)


class RoboOrienteering2018:
    EMERGENCY_STOP = 0x0001

    def __init__(self, config, bus):
        self.bus = bus
        self.maxspeed = config['maxspeed']
        self.goals = [latlon2xy(lat, lon) for lat, lon in config['waypoints']]
        self.time = None
        self.last_position = None  # (lon, lat) in milliseconds
        self.last_imu_yaw = None  # magnetic north in degrees
        self.status = None
        self.steering_status = None
        self.cmd = [0, 0]
        self.monitors = []
        self.last_position_angle = None  # for angle computation from dGPS

    def update(self):
        packet = self.bus.listen()
        if packet is not None:
#            print('RO', packet)
            timestamp, channel, data = packet
            self.time = timestamp
            if channel == 'position':
                self.last_position = data
            elif channel == 'orientation':
                (yaw, pitch, roll), (magx, y, z), (accx, y, z), (gyrox, y, z) = data
                self.last_imu_yaw = yaw
            elif channel == 'status':  # i.e. I can drive only spider??
                self.status, self.steering_status = data
                self.bus.publish('move', self.cmd)
            for monitor_update in self.monitors:
                monitor_update(self)

    def set_speed(self, speed, angular_speed):
        self.cmd = [speed, angular_speed]

    def start(self):
        pass

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

    def play0(self):
        self.wait(timedelta(seconds=1))
        self.set_speed(1, 50)
        self.wait(timedelta(seconds=5))
        self.set_speed(10, 0)
        self.wait(timedelta(seconds=5))
        self.set_speed(0, 0)
        self.wait(timedelta(seconds=1))

    def play(self):
        print("Waiting for valid GPS position...")
        while self.last_position is None or self.last_position == INVALID_COORDINATES:
            self.update()
        print(self.last_position)

        print("Wait for valid IMU...")
        while self.last_imu_yaw is None:
            self.update()
        print(self.last_imu_yaw)

        print("Wait for steering position...")
        while self.steering_status is None:
            self.update()
        print(self.steering_status)

        print("Ready", self.goals)
        try:
            with EmergencyStopMonitor(self):
                for goal in self.goals:
                    print("Goal at %.2fm" % geo_length(self.last_position, goal))
                    print("Heading %.1fdeg, imu" % math.degrees(geo_angle(self.last_position, goal)), self.last_imu_yaw)
                    self.navigate_to_goal(goal, timedelta(seconds=200))
        except EmergencyStopException:
            print("EMERGENCY STOP (wait 3s)")
            self.set_speed(0, 0)
            start_time = self.time
            while self.time - start_time < timedelta(seconds=3):
                self.set_speed(0, 0)
                self.update()

    def navigate_to_goal(self, goal, timeout):
        start_time = self.time
        self.last_position_angle = self.last_position
        gps_angle = None
        while geo_length(self.last_position, goal) > 1.0 and self.time - start_time < timeout:
            step = geo_length(self.last_position, self.last_position_angle)
            if step > 1.0:
                gps_angle = normalizeAnglePIPI(geo_angle(self.last_position_angle, self.last_position))
                print('step', step, math.degrees(gps_angle))
                self.last_position_angle = self.last_position

            desired_heading = normalizeAnglePIPI(geo_angle(self.last_position, goal))
            if gps_angle is None or self.steering_status[0] is None:
                spider_heading = normalizeAnglePIPI(math.radians(180 - self.last_imu_yaw - 35.5))
                wheel_heading = normalizeAnglePIPI(desired_heading-spider_heading)
            else:
                wheel_heading = math.radians(-360*self.steering_status[0]/512)
                wheel_heading = normalizeAnglePIPI(desired_heading - gps_angle + wheel_heading)

            desired_steering = int(-512*math.degrees(wheel_heading)/360.0)

            speed = self.maxspeed
            if self.steering_status[0] is None:
                speed = 1  # in in place
            else:
                 d = desired_steering - self.steering_status[0]
                 if d > 256:
                     d -= 512
                 elif d < -256:
                     d += 512
                 if abs(d) > 30:
                     speed = 1  # turn in place (II.)

            self.set_speed(speed, desired_steering)

            prev_time = self.time
            self.update()

            if int(prev_time.total_seconds()) != int(self.time.total_seconds()):
                print(self.time, geo_length(self.last_position, goal), self.last_imu_yaw, self.steering_status)

        print("STOP (3s)")
        self.set_speed(0, 0)
        start_time = self.time
        while self.time - start_time < timedelta(seconds=3):
            self.set_speed(0, 0)
            self.update()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='RoboOrienteering 2018')
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
        game = replay(args, application=RoboOrienteering2018)
        game.play()

    elif args.command == 'run':
        log = LogWriter(prefix='ro2018-', note=str(sys.argv))
        config = config_load(*args.config)
        log.write(0, bytes(str(config), 'ascii'))  # write configuration
        robot = Robot(config=config['robot'], logger=log, application=RoboOrienteering2018)
        game = robot.modules['app']  # TODO nicer reference
        robot.start()
        game.play()
        robot.finish()
    else:
        assert False, args.command  # unsupported command

# vim: expandtab sw=4 ts=4
