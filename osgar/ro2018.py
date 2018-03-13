"""
  RoboOrienteering 2018 - experiments with lib.logger and robot container
"""

import argparse
import sys
import math
from ast import literal_eval
from datetime import timedelta

import numpy as np

from osgar.lib.logger import LogWriter, LogReader
from osgar.lib.config import Config
from osgar.drivers import all_drivers
from osgar.robot import Robot

from osgar.drivers.gps import INVALID_COORDINATES, GPS_MSG_DTYPE
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


class RoboOrienteering2018:
    def __init__(self, config, bus):
        self.bus = bus
        self.goal = (51749517, 180462688 - 1000)  # TODO extra configuration
        self.time = None
        self.last_position = None  # (lon, lat) in milliseconds
        self.last_imu_yaw = None  # magnetic north in degrees
        self.status = None
        self.steering_status = None
        self.cmd = (0, 0)

    def update(self):
        packet = self.bus.listen()
        if packet is not None:
#            print('RO', packet)
            timestamp, channel, data = packet
            self.time = timestamp
            if channel == 'position':
                self.last_position = (data['lon'], data['lat'])
            elif channel == 'orientation':
                (yaw, pitch, roll), (magx, y, z), (accx, y, z), (gyrox, y, z) = data
                self.last_imu_yaw = yaw
            elif channel == 'status':  # i.e. I can drive only spider??
                self.status, self.steering_status = data
                self.bus.publish('move', self.cmd)

    def set_speed(self, speed, angular_speed):
        self.cmd = (speed, angular_speed)

    def start(self):
        pass

    def request_stop(self):
        self.bus.shutdown()

    def join(self):
        pass

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

        print("Ready")
        print("Goal at %.2fm" % geo_length(self.last_position, self.goal))

        print("Heading %.1fdeg, imu" % math.degrees(geo_angle(self.last_position, self.goal)), self.last_imu_yaw)
        
        start_time = self.time
        while geo_length(self.last_position, self.goal) > 1.0 and self.time - start_time < timedelta(seconds=20):
            desired_heading = normalizeAnglePIPI(geo_angle(self.last_position, self.goal))
            spider_heading = normalizeAnglePIPI(math.radians(180 + self.last_imu_yaw))
            wheel_heading = normalizeAnglePIPI(desired_heading-spider_heading)

            desired_steering = int(-512*math.degrees(wheel_heading)/360.0)
            self.set_speed(1, desired_steering)

            prev_time = self.time
            self.update()
            if int(prev_time.total_seconds()) != int(self.time.total_seconds()):
                print(self.time, geo_length(self.last_position, self.goal), self.last_imu_yaw, self.steering_status)

        print("STOP (3s)")
        self.set_speed(0, 0)
        start_time = self.time
        while self.time - start_time < timedelta(seconds=3):
            self.set_speed(0, 0)
            self.update()


# move to drivers/bus??
class LogBusHandler:
    def __init__(self, log, inputs, outputs):
        if outputs is None:
            self.reader = log.read_gen(inputs.keys())
        else:
            self.reader = log.read_gen(list(inputs.keys()) + list(outputs.keys()))
        self.inputs = inputs
        self.outputs = outputs

    def listen(self):
        dt, stream_id, data = next(self.reader)
        channel = self.inputs[stream_id]
        try:
            return dt, channel, literal_eval(data.decode('ascii'))
        except ValueError:
            return dt, channel, np.frombuffer(data, dtype=GPS_MSG_DTYPE)

    def publish(self, channel, data):
        if self.outputs is not None:
            dt, stream_id, raw_data = next(self.reader)
            channel = self.outputs[stream_id]
            ref_data = literal_eval(raw_data.decode('ascii'))
            assert data == ref_data, (data, ref_data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='RoboOrienteering 2018')
    subparsers = parser.add_subparsers(help='sub-command help', dest='command')
    subparsers.required = True
    parser_run = subparsers.add_parser('run', help='run on real HW')
    parser_run.add_argument('config', help='configuration file')
    parser_run.add_argument('--note', help='add description')

    parser_replay = subparsers.add_parser('replay', help='replay from logfile')
    parser_replay.add_argument('logfile', help='recorded log file')
    parser_replay.add_argument('--force', '-F', dest='force', action='store_true', help='force replay even for failing output asserts')
    args = parser.parse_args()

    if args.command == 'replay':
        log = LogReader(args.logfile)
        print(next(log.read_gen(0))[-1])  # old arguments
        config_str = next(log.read_gen(0))[-1]
        config = literal_eval(config_str.decode('ascii'))
        if args.force:
            outputs = None
        else:
            outputs = {1:'move'}
        bus = LogBusHandler(log,
                            inputs={2:'position', 4:'orientation', 7:'status'},
                            outputs=outputs)  # TODO map names
        game = RoboOrienteering2018(config={}, bus=bus)
        game.play()

    elif args.command == 'run':
        log = LogWriter(prefix='ro2018-', note=str(sys.argv))
        config = Config.load(args.config)
        log.write(0, bytes(str(config.data), 'ascii'))  # write configuration
        robot = Robot(config=config.data['robot'], logger=log, application=RoboOrienteering2018)
        game = robot.modules['app']  # TODO nicer reference
        robot.start()
        game.play()
        robot.finish()
    else:
        assert False, args.command  # unsupported command

# vim: expandtab sw=4 ts=4
