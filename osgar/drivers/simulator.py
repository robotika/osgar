"""
  Simple Spider Simulator
"""
import math
from threading import Thread
from time import sleep
from datetime import timedelta

from osgar.lib.mathex import normalizeAnglePIPI
from osgar.drivers.bus import BusShutdownException


class SpiderSimulator(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)
        self.bus = bus
        self.ref_position = config['position']  # WGS84, milliseconds, [lon, lat]
        self.duration = timedelta(seconds=config.get('duration', 300))  # default 5min
        self.rel_pose = (0, 0, 0)  # rel pose to GPS position
        self.wheel_angle = 0.0  # math orientation, radians
        self.status = 0x3  # running
        self.desired_speed, self.desired_wheel_angle = None, None

    def run(self):
        try:
            self.bus.publish('status', [0x0, None])  # trigger "pump"
            self.bus.publish('position', self.get_position())
            self.bus.publish('orientation', [[0, 0, 0]]*4)  # "valid" IMU
            while True:
                timestamp, channel, data = self.bus.listen()
#                print(timestamp, channel, data)
                if timestamp > self.duration:
                    self.status = 0x0  # STOP
                if channel == 'move':
                    self.desired_speed, raw_angle = data
                    self.desired_wheel_angle = -math.pi * raw_angle / 256.0
                    self.bus.publish('status', [self.status, self.get_wheels_angles_raw()])

                    # update position at 10Hz
                    if self.status & 0x8000 == 0x0:
                        self.bus.publish('position', self.get_position())
                else:
                    assert False, channel  # unsupported channel

                dt = 0.05
                sleep(dt)  # slow down basic cycle (for now)
                self.step(dt)
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

    def step(self, dt):
        if abs(self.desired_speed) > 0:
            dist = dt  # i.e. 1m/s
            if abs(self.desired_speed) <= 1.0:
                dist = 0.0  # turn in place
            x, y, heading = self.rel_pose
            a = heading + self.wheel_angle
            self.rel_pose = x + dist*math.cos(a), y + dist*math.sin(a), heading

            diff = normalizeAnglePIPI(self.desired_wheel_angle - self.wheel_angle)
            if abs(diff) > math.radians(1):
                if diff > 0:
                    corr = math.radians(10)  # 10deg/s
                else:
                    corr = math.radians(-10)
                self.wheel_angle += dt*corr
        self.status ^= 0x8000

    def get_position(self):
        "combine pose with ref GPS position"
        lon, lat = self.ref_position
        x_scale = math.cos(math.radians(lon/3600000))
        scale = 40000000/(360*3600000)
        x, y, __ = self.rel_pose
        return [int(lon + x/(scale*x_scale)), int(lat + y/scale)]

    def get_wheels_angles_raw(self):
        # Spider has 512 ticks per 360 degrees, clockwise
        raw = (-int(256 * self.wheel_angle / math.pi)) & 0x1FF
        return [raw] * 4

# vim: expandtab sw=4 ts=4

