import collections
import datetime
import inspect
import math

from threading import Thread, RLock

from osgar.lib import quaternion

try:
    from colored import stylize, fg
except ImportError:
    def stylize(what, style):
        return what
    def fg(color_name):
        return ""

_saved_print = print

def verbose(ok):
    print(ok)
    if ok:
        globals()['print'] = _saved_print
    else:
        globals()['print'] = lambda *args: None


def log(*args):
    up = inspect.currentframe().f_back
    caller = up.f_code.co_name
    caller_class = up.f_locals['self'].__class__
    color_name = getattr(caller_class, caller).color
    print(stylize(f"{caller_class.__name__}.{caller}", fg(color_name)), *args)


def clip(min_value, value, max_value):
    return max(min(value, max_value), min_value)


class Color:
    def __getattr__(self, color_name):
        def a(method):
            method.color = color_name
            return method
        return a


color = Color()


class SimLogger:

    def __init__(self):
        self.lock = RLock()
        self.start_time = datetime.datetime.now(datetime.timezone.utc)
        self.names = ["sys"]

    def register(self, name):
        self.names.append(name)
        return len(self.names)-1

    @color.dodger_blue_1
    def write(self, stream_id, data, dt=None):
        #log(self.names[stream_id], data)
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

    @color.green
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

    @color.red
    def on_default(self, dt, channel, data):
        log(dt, channel, data)

    @color.gold_1
    def on_request_origin(self, dt, channel, data):
        log(dt, channel, data)
        # start sending origin data until robot arrives within 0.3m from (0,0,0)
        corrected = [rr - oo for rr, oo in zip(self.xyz, self.origin)]
        self.bus.publish('origin', [b'X0F100L', *corrected, *self.orientation])
        self.bus.publish('scan', [0] * 100)

    @color.turquoise_4
    def on_pose(self, dt, channel, data):
        if channel == 'pose2d':
            out = str(data)
        elif channel == 'pose3d':
            out0 = "[" + ', '.join([f"{p:.2f}" for p in data[0]]) + "]"
            out1 = " [" + ', '.join([f"{p:.2f}" for p in data[1]]) + "]"
            out = out0 + out1
        log(dt, channel, out)

    @color.sea_green_3
    def on_desired_speed(self, dt, channel, data):
        forward = data[0]/1000.
        angular = data[1]/100.
        diff_forward = clip(-0.1, forward - self.speed_forward, 0.1)
        diff_angular = clip(-10.0, angular - self.speed_angular, 10.0)
        #print(diff_forward, diff_angular)
        self.speed_forward += diff_forward
        self.speed_angular += diff_angular
        log("desired", f"forward {forward:.2f} m/s", f"angular {angular:.2f} deg/s")
        log("current", f"forward {self.speed_forward:.2f} m/s", f"angular {self.speed_angular:.2f} deg/s")
        dtime = 0.1
        dist = dtime * self.speed_forward
        dist3d = quaternion.rotate_vector([dist, 0, 0], self.orientation)
        self.xyz = [a + b for a, b in zip(self.xyz, dist3d)]
        turn = quaternion.from_axis_angle((0,0,1), math.radians(dtime * self.speed_angular))
        self.orientation = quaternion.multiply(self.orientation, turn)
        self.time += datetime.timedelta(seconds=dtime)
        heading_centidegrees = round(math.degrees(quaternion.heading(self.orientation))*100)
        x_mm, y_mm, z_mm = [round(c*1000) for c in self.xyz]
        self.bus.publish('orientation', self.orientation)
        self.bus.publish('rot', [heading_centidegrees, 0, 0])
        self.bus.publish('pose2d', [x_mm, y_mm, heading_centidegrees])
        self.bus.publish('sim_time_sec', self.time.total_seconds())
        self.bus.publish('scan', [0] * 100)

        #log("++++", x, y, math.degrees(heading))
