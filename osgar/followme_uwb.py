"""
  UWB (pozyx) version of Follow Me (primary target Eduro robot)
"""
import math
from datetime import timedelta

from osgar.node import Node
from osgar.followme import EmergencyStopException

# Notes:
# Expects 3 anchors mounted on Eduro robot


class FollowMeUWB(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed')
        self.last_position = [0, 0, 0]  # proper should be None, but we really start from zero
        self.raise_exception_on_stop = False
        self.verbose = False

        self.left_range = None
        self.right_range = None
        self.back_range = None
        self.debug_arr = []

    def on_scan(self, data):
        pass  # ignore for now

    def on_encoders(self, data):
        pass  # ignore for now

    def on_pose2d(self, data):
        pass  # ignore for now

    def on_pozyx_range(self, data):
        # [1, 3431, 3411, [2777589, 357, -78]]
        if data[0] == 1:
            tag = 0x6827
            if data[1] == tag or data[2] == tag:
                src = data[1] if data[2] == tag else data[2]
                dist = data[3][1] / 1000
                if src == 0xD53:
                    self.left_range = dist
                elif src == 0xD67:
                    self.right_range = dist
                else:
                    assert src is None, src
                    self.back_range = dist

                if self.left_range is not None and self.right_range is not None:
                    diff = self.left_range - self.right_range
                    if self.verbose:
                        print(diff)
                        self.debug_arr.append((self.time.total_seconds(), diff))
                    angular_speed = math.radians(10)
                    if abs(diff) < 0.05:
                        self.send_speed_cmd(0.0, 0.0)
                    elif diff > 0:
                        self.send_speed_cmd(0.0, -angular_speed)
                    else:
                        self.send_speed_cmd(0.0, angular_speed)

    def on_pozyx_gpio(self, data):
        pass  # ignore for now

    def on_buttons(self, data):
        pass  # ignore for now

    def update(self):  # yes, refactoring to some common node would be nice!
        channel = super().update()  # define self.time
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # unknown

    def draw(self):
        import matplotlib.pyplot as plt
        t = [a[0] for a in self.debug_arr]
        x = [a[1] for a in self.debug_arr]
        line = plt.plot(t, x, '-o', linewidth=2, label=f'diff')

        plt.xlabel('time (s)')
        plt.ylabel('distance diff (m)')
        plt.legend()
        plt.show()



    def XXXupdate(self):
        channel = super().update()  # define self.time
        if channel == 'pose2d':
            self.last_position = self.pose2d
        elif channel == 'scan':
            if self.verbose:
                print(min_dist(self.scan)/1000.0)
            self.last_scan = self.scan
        elif channel == 'emergency_stop':
            if self.raise_exception_on_stop and self.emergency_stop:
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

    def followme_orig(self):
        print("Follow Me!")

        # SCAN_SIZE = 811  # TODO config
        SCAN_SIZE = 271
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

    def XXXfollowme(self):
        print("Follow Me UWB!")

    def XXXrun(self):
        try:
            self.raise_exception_on_stop = True
            self.followme()
        except EmergencyStopException:
            print('!!!Emergency STOP!!!')
            self.raise_exception_on_stop = False
            self.send_speed_cmd(0.0, 0.0)
            self.wait(timedelta(seconds=1))


if __name__ == "__main__":
    from osgar.launcher import launch

    launch(app=FollowMe, description='Follow Me', prefix='followme-')

# vim: expandtab sw=4 ts=4
