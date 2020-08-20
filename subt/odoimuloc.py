import collections
import logging
import math
import osgar.node

import osgar.lib.quaternion as quaternion

g_logger = logging.getLogger(__name__)

Pose2d = collections.namedtuple("Pose2d", ("x", "y", "heading"))

class Localization(osgar.node.Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        # outputs
        bus.register('pose3d')
        # inputs: origin, orientation, odom
        self.xyz = None # 3D position updated using odometry dist and imu orientation, defined when 'origin' received
        self.orientation = None # not defined until first 'orientation' received
        self.last_odom = None
        self.origin_xyz = None
        self.origin_orientation = None
        self.origin_error = False

    def on_origin(self, data):
        if len(data) == 8:
            self.xyz = data[1:4]
            self.origin_xyz = data[1:4]
            self.origin_orientation = data[4:]
        else:
            self.origin_error = True

    def on_orientation(self, orientation):
        self.orientation = orientation

    def on_odom(self, pose2d):
        x, y, heading = pose2d
        assert self.xyz is not None
        if self.orientation is None:
            return

        odom = Pose2d(x / 1000.0, y / 1000.0, math.radians(heading / 100.0))
        if self.last_odom is not None:
            dist = math.hypot(odom.x - self.last_odom.x, odom.y - self.last_odom.y)
            direction = ((odom.x - self.last_odom.x) * math.cos(self.last_odom.heading) +
                         (odom.y - self.last_odom.y) * math.sin(self.last_odom.heading))
            if direction < 0:
                dist = -dist
        else:
            dist = 0.0

        self.last_odom = odom
        dist3d = quaternion.rotate_vector([dist, 0, 0], self.orientation)
        self.xyz = [a + b for a, b in zip(self.xyz, dist3d)]
        self.bus.publish('pose3d', [self.xyz, self.orientation])

    def run(self):
        try:
            # wait for valid origin
            while True:
                dt, channel, data = self.bus.listen()
                if channel == "origin":
                    self.on_origin(data)
                    if self.xyz is not None:
                        break

            while True:
                dt, channel, data = self.bus.listen()
                self.time = dt
                if channel == "orientation":
                    self.on_orientation(data)
                elif channel == "odom":
                    self.on_odom(data)
                elif channel == "origin":
                    pass
                else:
                    assert False, "unknown input channel"
        except osgar.bus.BusShutdownException:
            pass



