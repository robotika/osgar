"""
  UWB (pozyx) version of Follow Me (primary target Eduro robot)
"""
import math
from datetime import timedelta
from statistics import median

from osgar.node import Node
from osgar.followme import EmergencyStopException, min_dist

# Notes:
# Expects 3 anchors mounted on Eduro robot

FILTER_SIZE = 10

MODE_FOLLOW_UWB = 1
MODE_FOLLOW_PATH = 2

DIST_STEP = 0.2


class FollowMeUWB(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed')
        self.last_position = [0, 0, 0]  # proper should be None, but we really start from zero
        self.raise_exception_on_stop = False
        self.verbose = False
        self.last_min_dist = None  # unknown
        self.follow_enabled = None  # unknown

        self.left_id = int(config['left_id'], 16)
        self.right_id = int(config['right_id'], 16)

        self.left_range = None
        self.right_range = None
        self.left_range_arr = []
        self.right_range_arr = []
        self.back_range = None
        self.debug_arr = []
        self.navigation_mode = None  # not selected
        self.path = []

    def on_scan(self, data):
        assert len(data) == 271, len(data)
        self.last_min_dist = min_dist(data[45:-45])

    def on_encoders(self, data):
        pass  # ignore for now

    def on_pose2d(self, data):
        if self.navigation_mode == MODE_FOLLOW_UWB:
            if len(self.path) == 0 or self.path[-1] != data:
                self.path.append(data)
        elif self.navigation_mode == MODE_FOLLOW_PATH:
            for i, p in enumerate(self.path):
                d = math.hypot(data[0] - p[0], data[1] - p[1]) / 1000.0
                if d < DIST_STEP:
                    break
            if i == 0:
                self.send_speed_cmd(0, 0)
            else:
                speed = 0.0
                diff = (data[2] - p[2] + 18000) % 36000
                if abs(diff) < 100:
                    angular_speed = 0
                    speed = 0.1
                elif diff < 0:
                    angular_speed = math.radians(10)
                else:
                    angular_speed = -math.radians(10)
                self.send_speed_cmd(speed, angular_speed)

    def on_pozyx_range(self, data):
        if self.navigation_mode != MODE_FOLLOW_UWB:
            return

        # [1, 3431, 3411, [2777589, 357, -78]]
        if data[0] == 1:
            tag = 0x6827
            if data[1] == tag or data[2] == tag:
                src = data[1] if data[2] == tag else data[2]
                dist = data[3][1] / 1000
                if src == self.left_id:
                    self.left_range_arr.append(dist)
                    self.left_range_arr = self.left_range_arr[-FILTER_SIZE:]
                    self.left_range = median(self.left_range_arr)
                elif src == self.right_id:
                    self.right_range_arr.append(dist)
                    self.right_range_arr = self.right_range_arr[-FILTER_SIZE:]
                    self.right_range = median(self.right_range_arr)
                else:
                    assert src is None, src
                    self.back_range = dist

                if self.left_range is not None and self.right_range is not None:
                    diff = self.left_range - self.right_range
                    dist = (self.left_range + self.right_range)/2
                    if self.verbose:
                        print(diff, dist)
                        self.debug_arr.append((self.time.total_seconds(), diff))
                    angular_speed = math.radians(10)
                    speed = 0.0
                    if dist > 1.2:
                        speed = min(0.5, 0.1 + (dist - 1.2) * 0.4)
                    if self.last_min_dist is not None and self.last_min_dist < 700:
                        speed = 0.0

                    if self.follow_enabled:
                        if abs(diff) < 0.05:
                            self.send_speed_cmd(speed, 0.0)
                        elif diff > 0:
                            self.send_speed_cmd(speed, -angular_speed)
                        else:
                            self.send_speed_cmd(speed, angular_speed)
                    else:
                        self.send_speed_cmd(0, 0)

    def on_tag_sensor(self, data):
        row = [int(x) for x in data[2].split(',')]
#        self.debug_arr.append((self.time.total_seconds(), row[-7:-4]))

    def on_pozyx_gpio(self, data):
        # [1, 26663, 0]
        valid, device_id, digital_input = data
        if valid:
            self.follow_enabled = (digital_input == 0)

    def on_buttons(self, data):
        assert 'blue_selected' in data, data
        if data['blue_selected']:
            self.navigation_mode = MODE_FOLLOW_PATH
        else:
            self.navigation_mode = MODE_FOLLOW_UWB

    def update(self):  # yes, refactoring to some common node would be nice!
        channel = super().update()  # define self.time
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # unknown

    def draw0(self):
        import matplotlib.pyplot as plt
        t = [a[0] for a in self.debug_arr]
        x = [a[1] for a in self.debug_arr]
        line = plt.plot(t, x, '-o', linewidth=2, label=f'diff')

        plt.xlabel('time (s)')
        plt.ylabel('distance diff (m)')
        plt.legend()
        plt.show()

    def draw(self):
        import matplotlib.pyplot as plt
        x = [a[0]/1000.0 for a in self.path]
        y = [a[1]/1000.0 for a in self.path]
        line = plt.plot(x, y, '-o', linewidth=2, label='path')
        plt.axes().set_aspect('equal', 'datalim')
        plt.legend()
        plt.show()

    def send_speed_cmd(self, speed, angular_speed):
        return self.bus.publish('desired_speed',
                [round(speed*1000), round(math.degrees(angular_speed)*100)])

# vim: expandtab sw=4 ts=4
