"""
  Follow Path - ver0 static, ver1 dynamically updated via msg "path'
"""
import math

from osgar.lib.route import Route as GPSRoute, DummyConvertor
from osgar.lib.mathex import normalizeAnglePIPI
from osgar.node import Node
from osgar.bus import BusShutdownException
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
        self.finished = False

    def control(self, pose):
        """
        Based on current "pose2d" robot position and "self.route" return desired speed and angular speed.

        The current implementation takes nearest point on the route and predicts next position following
        the route (now hardcoded 20cm). Then calculate absolute direction (via following another 10cm)
        and define angular speed to reach this absolute direction within 1 second. If the nearest position
        is already by the end of route then stop.

        Note, that there is no correction based on signed distance from route.
        """
        first, second = self.route.routeSplit(pose[:2])
        if len(second) <= 1:
            self.finished = True
            return 0, 0
        pt = Route(second).pointAtDist(dist=0.2)  # maybe speed dependent
        pt2 = Route(second).pointAtDist(dist=0.2+0.1)
        if math.hypot(pt2[1]-pt[1], pt2[0]-pt[0]) < 0.001:
            # at the very end, or not defined angle
            self.finished = True
            return 0, 0
        angle = math.atan2(pt2[1]-pt[1], pt2[0]-pt[0])
        if self.verbose:
            print(self.time, second, pt, angle)
        return self.max_speed, normalizeAnglePIPI(angle - pose[2])

    def on_pose2d(self, data):
        x, y, heading = data
        self.last_position = [x / 1000.0, y / 1000.0, math.radians(heading / 100.0)]
        speed, angular_speed = self.control(self.last_position)
        if self.verbose:
            print(speed, angular_speed)
        self.send_speed_cmd(speed, angular_speed)
        if self.finished:
            raise BusShutdownException()

    def on_emergency_stop(self, data):
        if self.raise_exception_on_stop and data:
            raise EmergencyStopException()

    def send_speed_cmd(self, speed, angular_speed):
        return self.bus.publish(
            'desired_speed',
            [round(speed*1000), round(math.degrees(angular_speed)*100)]
        )

# vim: expandtab sw=4 ts=4
