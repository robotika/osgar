import collections
import datetime
import inspect
import math
import unittest

from unittest.mock import MagicMock
from threading import Thread, RLock

from osgar.lib import quaternion
from osgar.bus import Bus
from subt.main import SubTChallenge
from subt.trace import distance3D

try:
    from colored import stylize, fg
except ImportError:
    def stylize(what, style):
        return what
    def fg(color_name):
        return ""

import sys
if '-v' not in sys.argv or '--verbose' not in sys.argv:
    def print(*args):
        pass

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

        for _ in range(321):
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
        heading = quaternion.heading(self.orientation)
        x, y, z = self.xyz
        x += dist * math.cos(heading)
        y += dist * math.sin(heading)
        self.xyz = (x, y, z)
        turn = quaternion.from_axis_angle((0,0,1), math.radians(dtime * self.speed_angular))
        #print(self.time, turn)
        self.orientation = quaternion.multiply(self.orientation, turn)
        self.time += datetime.timedelta(seconds=dtime)

        self.bus.publish('orientation', self.orientation)
        self.bus.publish('rot', [round(math.degrees(heading)*100), 0, 0])
        self.bus.publish('pose2d', [round(x*1000), round(y*1000), round(math.degrees(heading)*100)])
        self.bus.publish('sim_time_sec', self.time.total_seconds())
        self.bus.publish('scan', [0] * 100)

        #log("++++", x, y, math.degrees(heading))


def entrance_reached(sim):
    corrected = [(rr - oo) for rr, oo in zip(sim.xyz, sim.origin)]
    goal = [2.5, 0, 0]
    if distance3D(corrected, goal) < 2:
        return True
    return False


class SubTChallengeTest(unittest.TestCase):

    def test_maybe_remember_artifact(self):
        config = {'max_speed': 0.5, 'walldist': 0.9, 'timeout': 600, 'symmetric': True,
                  'right_wall': True}
        bus = MagicMock()
        game = SubTChallenge(config, bus)

        artf_data = ['TYPE_BACKPACK', -1614, 1886]
        artf_xyz = (0, 0, 0)
        self.assertTrue(game.maybe_remember_artifact(artf_data, artf_xyz))

        # 2nd report should be ignored
        self.assertEqual(game.maybe_remember_artifact(artf_data, artf_xyz), False)

    def test_go_to_entrance(self):
        config = {'virtual_world': True, 'max_speed': 1.0, 'walldist': 0.8, 'timeout': 600, 'symmetric': False, 'right_wall': 'auto'}
        log = SimLogger()
        bus = Bus(log)
        app = SubTChallenge(config, bus.handle('app'))
        sim = Simulation(bus.handle('sim'), end_condition=entrance_reached)
        print("connecting:")
        for o in bus.handle('sim').out:
            print(f'  sim.{o} → app.{o}')
            bus.connect(f'sim.{o}', f'app.{o}')
        for o in bus.handle('app').out:
            print(f'  app.{o} → sim.{o}')
            bus.connect(f'app.{o}', f'sim.{o}')
        print("done.")
        app.start()
        sim.start()
        sim.join()
        app.request_stop()
        app.join()
        self.assertTrue(entrance_reached(sim))



# vim: expandtab sw=4 ts=4

