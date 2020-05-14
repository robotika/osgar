import collections
import datetime
import inspect
import logging
import math

import numpy as np

from threading import Thread, RLock

from osgar.lib import quaternion


g_logger = logging.getLogger(__name__)


def log(*args):
    up = inspect.currentframe().f_back
    caller = up.f_code.co_name
    caller_class = up.f_locals['self'].__class__
    msg = f"{caller_class.__name__}.{caller}, " + ", ".join(str(a) for a in args)
    g_logger.info(msg)


class SimLogger:

    def __init__(self):
        self.lock = RLock()
        self.start_time = datetime.datetime.now(datetime.timezone.utc)
        self.names = ["sys"]

    def register(self, name):
        self.names.append(name)
        return len(self.names)-1

    def write(self, stream_id, data, dt=None):
        dt = datetime.datetime.now(datetime.timezone.utc) - self.start_time
        return dt


class Simulation:

    def __init__(self, bus, end_condition):
        bus.register('scan', 'rot', 'orientation', 'sim_time_sec', 'origin', 'pose2d', 'acc', 'artf')
        self.bus = bus
        self.end_condition = end_condition
        self._handlers = collections.defaultdict(lambda: self.on_default)
        self.set_handler('request_origin', self.on_request_origin)
        self.set_handler('desired_speed', self.on_desired_speed)
        self.set_handler('pose2d', self.on_pose)
        self.set_handler('pose3d', self.on_pose)
        self.origin = [7.0, 3.0, 0.0]
        self.xyz = [0.0, 0.0, 0.0]
        self.orientation = quaternion.identity()
        self.speed_forward = 0.0
        self.speed_angular = 0.0
        self.time = datetime.timedelta()

    def start(self):
        self.thread = Thread(target=self.main)
        self.thread.start()

    def join(self, timeout=None):
        self.thread.join(timeout)

    def main(self):
        log("started")
        self.on_request_origin(datetime.timedelta(), "request_origin", True)

        for _ in range(500):
            packet = self.bus.listen()
            self.handle_incomming(*packet)
            if self.end_condition(self):
                return

    def set_handler(self, channel, handler):
        self._handlers[channel] = handler

    def handle_incomming(self, dt, channel, data):
        self._handlers[channel](dt, channel, data)

    def on_default(self, dt, channel, data):
        log(dt, channel, data)

    def on_request_origin(self, dt, channel, data):
        log(dt, channel, data)
        # start sending origin data until robot arrives within 0.3m from (0,0,0)
        corrected = [rr - oo for rr, oo in zip(self.xyz, self.origin)]
        self.bus.publish('origin', [b'X0F100L', *corrected, *self.orientation])
        self.bus.publish('scan', [0] * 100)

    def on_pose(self, dt, channel, data):
        if channel == 'pose2d':
            out = str(data)
        elif channel == 'pose3d':
            out0 = "[" + ', '.join([f"{p:.2f}" for p in data[0]]) + "]"
            out1 = " [" + ', '.join([f"{p:.2f}" for p in data[1]]) + "]"
            out = out0 + out1
        log(dt, channel, out)

    def on_desired_speed(self, dt, channel, data):
        forward = data[0]/1000.               # from millimeters per second to meters per second
        angular = math.radians(data[1]/100.)  # from centidegrees per second to radians per second
        dtime = 0.1                           # simulation step time in seconds
        max_acc_forward = 1                   # meters per sec**2
        max_acc_angular = 1                   # radians per sec**2
        acc_forward = np.clip(-max_acc_forward, (forward - self.speed_forward)/dtime, max_acc_forward)
        acc_angular = np.clip(-max_acc_angular, (angular - self.speed_angular)/dtime, max_acc_angular)
        self.speed_forward += acc_forward * dtime
        self.speed_angular += acc_angular * dtime
        log("desired", f"forward {forward:.2f} m/s", f"angular {math.degrees(angular):.2f} deg/s")
        log("current", f"forward {self.speed_forward:.2f} m/s", f"angular {math.degrees(self.speed_angular):.2f} deg/s")
        dist = dtime * self.speed_forward
        dist3d = quaternion.rotate_vector([dist, 0, 0], self.orientation)
        self.xyz = [a + b for a, b in zip(self.xyz, dist3d)]
        turn = quaternion.from_axis_angle((0,0,1), dtime * self.speed_angular)
        self.orientation = quaternion.multiply(self.orientation, turn)
        self.time += datetime.timedelta(seconds=dtime)
        heading_centidegrees = round(math.degrees(quaternion.heading(self.orientation))*100)
        x_mm, y_mm, z_mm = [round(c*1000) for c in self.xyz]
        self.bus.publish('orientation', self.orientation)
        self.bus.publish('rot', [heading_centidegrees, 0, 0])
        self.bus.publish('pose2d', [x_mm, y_mm, heading_centidegrees])
        self.bus.publish('sim_time_sec', self.time.total_seconds())
        self.bus.publish('scan', [0] * 100)

