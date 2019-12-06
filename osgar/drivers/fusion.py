"""
  pose2d fusion
"""
import math

from osgar.node import Node


class FusionPose2d(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('pose2d')
        self.yaw, self.pitch, self.roll = 0, 0, 0
        self.xyz = 0, 0, 0
        self.last_position = None

    def update(self):
        channel = super().update()
        if channel == 'pose2d':
            x, y, heading = self.pose2d
            pose = (x/1000.0, y/1000.0, math.radians(heading/100.0))
            if self.last_position is not None:
                dist = math.hypot(pose[0] - self.last_position[0], pose[1] - self.last_position[1])
                direction = ((pose[0] - self.last_position[0]) * math.cos(self.last_position[2]) +
                             (pose[1] - self.last_position[1]) * math.sin(self.last_position[2]))
                if direction < 0:
                    dist = -dist
            else:
                dist = 0.0
            self.last_position = pose

            x, y, z = self.xyz
            x += math.cos(self.pitch) * math.cos(self.yaw) * dist
            y += math.cos(self.pitch) * math.sin(self.yaw) * dist
            z += math.sin(self.pitch) * dist
            self.bus.publish('pose2d', [round(x*1000), round(y*1000),
                                        round(math.degrees(self.yaw)*100)])
            self.xyz = x, y, z
        elif channel == 'rotation':
            self.yaw, self.pitch, self.roll = [math.radians(x/100) for x in self.rotation]
        return channel

# vim: expandtab sw=4 ts=4
