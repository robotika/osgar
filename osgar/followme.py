"""
  Follow Me (primary target Eduro robot)
"""
import math
from datetime import timedelta

from osgar.node import Node

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


class FollowMe(Node):
    PUSH_ACTION = 'push'

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed')
        self.last_position = [0, 0, 0]  # proper should be None, but we really start from zero
        self.raise_exception_on_stop = False
        self.verbose = False
        self.last_scan = None
        self.scan_size = config.get('scan_size', 271)
        self.scan_fov_deg = config.get('scan_fov_deg', 270)
        self.max_speed = config.get('max_speed', 0.5)  # m/s
        self.max_dist_limit = config.get('max_dist_limit', 1.3)  # m
        self.desired_dist = config.get('desired_dist', 0.4)  # m
        self.action = config.get('action', 'follow')
        assert self.action in ['follow', self.PUSH_ACTION], self.action
        self.debug_arr = []

    def on_pose2d(self, data):
        self.last_position = data

    def on_scan(self, data):
        if self.verbose:
            print(self.time, 'min_dist', min_dist(data) / 1000.0)
        self.last_scan = data
        if self.action == self.PUSH_ACTION:
            mid_size = len(data)//2
            self.last_scan = data[mid_size:] + data[:mid_size]

    def on_emergency_stop(self, data):
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

    def followme_step(self, scan, index, ):

        # constants could be in __init__()
        SCAN_SIZE = self.scan_size
        SCANS_PER_DEG = abs(SCAN_SIZE//self.scan_fov_deg)  # FOV can be negative for flipped lidar

        # limit tracking to front 180deg only due to mounting (back laser is blocked by robot body)
        LIMIT_LOW = 0  # SCAN_SIZE//6
        LIMIT_HIGH = SCAN_SIZE  # 5*SCAN_SIZE//6
        CLOSE_REFLECTIONS = 10  # ignore readings closer than 10mm, where 0 = infinite (no response)

        # obsolete, to be defined by robot shape, this fits to Eduro only
        thresholds = []
        for i in range(SCAN_SIZE):
            if LIMIT_LOW <= i <= LIMIT_HIGH:
                deg = -self.scan_fov_deg/2 + self.scan_fov_deg * i / SCAN_SIZE
                rad = math.radians(deg)
                thresh = 1000 * (0.17 + 0.17 * max(0, math.cos(rad)))  # [mm]
            else:
                thresh = 0
            thresholds.append(thresh)

        masterAngleOffset = 0  # TODO

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
        maxnear = min((x for x in self.last_scan if x > CLOSE_REFLECTIONS)) / 1000.0

        if self.verbose:
            print(self.time, 'near', near, index, (low, high), maxnear)

        if near > self.max_dist_limit or any(
                x < thresh for (x, thresh) in zip(self.last_scan, thresholds) if x > CLOSE_REFLECTIONS):
            speed, rot, angle = 0, 0, None
        else:
            angle = math.radians(self.scan_fov_deg * index / SCAN_SIZE - self.scan_fov_deg / 2) + masterAngleOffset
            desiredAngle = 0
            desiredDistance = self.desired_dist
            #                    speed = 0.2 + 2 * (near - desiredDistance)
            speed = near - desiredDistance
            #                    rot = 1.5 * (angle - desiredAngle)
            rot = angle - desiredAngle
            if speed < 0:
                speed = 0
            if speed > self.max_speed:
                speed = self.max_speed
        return speed, rot, index

    def followme(self):
        print(f"{self.action} me!")

        SCAN_SIZE = self.scan_size
        SCANS_PER_DEG = abs(SCAN_SIZE//self.scan_fov_deg)  # FOV can be negative for flipped lidar

        # limit tracking to front 180deg only due to mounting (back laser is blocked by robot body)
        LIMIT_LOW = 0  # SCAN_SIZE//6
        LIMIT_HIGH = SCAN_SIZE  # 5*SCAN_SIZE//6
        CLOSE_REFLECTIONS = 10  # ignore readings closer than 10mm, where 0 = infinite (no response)

        thresholds = []
        for i in range(SCAN_SIZE):
            if LIMIT_LOW <= i <= LIMIT_HIGH:
                deg = -self.scan_fov_deg/2 + self.scan_fov_deg * i / SCAN_SIZE
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
                    print(self.time, 'near', near, index, (low, high), maxnear)

                if near > self.max_dist_limit or any(x < thresh for (x, thresh) in zip(self.last_scan, thresholds) if x > CLOSE_REFLECTIONS):
                    speed, rot, angle = 0, 0, None
                else:
                    angle = math.radians(self.scan_fov_deg * index/SCAN_SIZE - self.scan_fov_deg/2) + masterAngleOffset
                    desiredAngle = 0
                    desiredDistance = self.desired_dist
#                    speed = 0.2 + 2 * (near - desiredDistance)
                    speed = near - desiredDistance
                    if self.action == self.PUSH_ACTION:
                        # get the max speed at desired distance and slow down in both directions
                        speed = self.max_speed - abs(desiredDistance - near)  # TODO scaling to self.max_dist_limit??
                    else:
                        speed = near - desiredDistance

                    #                    rot = 1.5 * (angle - desiredAngle)
                    rot = angle - desiredAngle
                    if speed < 0:
                        speed = 0
                    if speed > self.max_speed:
                        speed = self.max_speed
                if self.verbose:
                    print(self.time, 'speed', speed, angle, rot)
                    self.debug_arr.append((self.time.total_seconds(), angle, near, speed))
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

    def draw(self):
        # https://www.perplexity.ai/search/how-to-draw-with-matplotlib-in-SiM5q62xQIGSni3t0OUpYw
        import matplotlib.pyplot as plt

        x, y1, y2, y3 = map(list, zip(*self.debug_arr))

        # Create a figure with 3 subplots stacked vertically
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 8))

        # Plot the first graph
        ax1.plot(x, [math.degrees(tmp) if tmp is not None else None for tmp in y1])
        ax1.set_title('Angle (deg)')

        # Plot the second graph
        ax2.plot(x, y2)
        ax2.set_title('Distance (m)')

        # Plot the third graph
        ax3.plot(x, y3)
        ax3.set_title('Speed (m/s)')

        # Adjust the spacing between subplots
        plt.subplots_adjust(hspace=0.5)

        plt.show()


if __name__ == "__main__":
    from osgar.launcher import launch

    launch(app=FollowMe, description='Follow Me', prefix='followme-')

# vim: expandtab sw=4 ts=4
