"""
  Follow Path - ver0 static, ver1 dynamically updated via msg "path'
"""
import math

from osgar.lib.route import Route as GPSRoute, DummyConvertor
from osgar.node import Node
from osgar.followme import EmergencyStopException


class Route(GPSRoute):
    """
    Override GPS defaults to normal planar case
    """
    def __init__(self, pts=[]):
        super().__init__(pts, conv=DummyConvertor(), isLoop=False)


class FollowPath(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed')
        self.last_position = [0, 0, 0]  # proper should be None, but we really start from zero
        self.route = Route(pts=config.get('path', []))
        self.max_speed = config.get('max_speed', 0.2)
        self.raise_exception_on_stop = False
        self.verbose = False

    def nearest(self, pose, path):
        if len(path) == 0:
            return None, None
        if len(path) == 1:
            return path[0], None
        # dense points? crossing lines?
        min_dist = None
        best = None, None
        x, y, heading = pose
        for i, p in enumerate(path):
            dist = math.hypot(p[0] - x, p[1] - y)
            if min_dist is None or dist < min_dist:
                best = i
                min_dist = dist
        if best == 0:
            return path[0], path[1]
        if best == len(path) - 1:
            return path[-1], None
        # choose before or after
        dist_before = math.hypot(path[best - 1][0] - x, path[best - 1][1] - y)
        dist_after = math.hypot(path[best + 1][0] - x, path[best + 1][1] - y)
        if dist_before < dist_after:
            return path[best - 1], path[best]
        else:
            return path[best], path[best + 1]

    def control(self, pose):
        first, second = self.route.routeSplit(pose[:2])
        if len(second) <= 1:
            return 0, 0
        pt = Route(second).pointAtDist(dist=0.2)  # maybe speed dependent
        angle = Route(second).turnAngleAt(pt, radius=0.1)
        return self.max_speed, pose[2] - angle

    def on_pose2d(self, data):
        x, y, heading = data
        self.last_position = [x / 1000.0, y / 1000.0, math.radians(heading / 100.0)]
        speed, angular_speed = self.control(self.last_position)
        if self.verbose:
            print(speed, angular_speed)
        self.send_speed_cmd(speed, angular_speed)

    def on_emergency_stop(self, data):
        if self.raise_exception_on_stop and data:
            raise EmergencyStopException()

    def draw(self):
        import matplotlib.pyplot as plt
        t = [a[0] for a in self.debug_arr]
        x = [a[1] for a in self.debug_arr]
        line = plt.plot(t, x, '-o', linewidth=2, label=f'diff')

        plt.xlabel('time (s)')
        plt.ylabel('distance diff (m)')
        plt.legend()
        plt.show()

    def send_speed_cmd(self, speed, angular_speed):
        return self.bus.publish(
            'desired_speed',
            [round(speed*1000), round(math.degrees(angular_speed)*100)]
        )

# vim: expandtab sw=4 ts=4
