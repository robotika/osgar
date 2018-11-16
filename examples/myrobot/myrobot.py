"""
  Example of robot diver outside OSGAR package
    - simulation of differential robot
"""
import math
from threading import Thread

from osgar.bus import BusShutdownException


class MyTimer(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)
        self.bus = bus
        self.sleep_time = config['sleep']

    def run(self):
        try:
            while self.bus.is_alive():
                self.bus.publish('tick', None)
                self.bus.sleep(self.sleep_time)

        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()


class MyRobot(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)
        self.bus = bus
        self.pose = (0, 0, 0)
        self.speed, self.angular_speed = 0, 0
        self.desired_speed, self.desired_angular_speed = 0, 0
        self.last_update = None

    def send_pose(self):
        x, y, heading = self.pose        
        self.bus.publish('pose2d', [round(x*1000), round(y*1000),
                                    round(math.degrees(heading)*100)])

    def update_pose(self, diff_time):
        self.speed = self.desired_speed  # TODO motion model
        self.angular_speed = self.desired_angular_speed
        t = diff_time

        x, y, heading = self.pose
        x += math.cos(heading) * self.speed * t
        y += math.sin(heading) * self.speed * t
        heading += self.angular_speed * t
        self.pose = (x, y, heading)

    def run(self):
        try:
            while True:
                dt, channel, data = self.bus.listen()
                if self.last_update is not None:
                    t = (dt - self.last_update).total_seconds()
                    self.update_pose(t)
                self.last_update = dt

                if channel == 'desired_speed':
                    self.desired_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)

                self.send_pose()

        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4

