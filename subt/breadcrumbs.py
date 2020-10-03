"""
  OSGAR breadcrumbs dispenser for Virtual
"""
from ast import literal_eval

from osgar.node import Node
from subt.trace import distance3D


class Breadcrumbs(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("deploy", "location")

        self.robot_name = config.get('robot_name')
        self.start_time = None  # unknown
        self.next_deploy_time = None
        self.step_time = config.get('step_sec')

        self.radius = config.get('radius')
        self.locations = [[0, 0, 0]]  # all locations + fake for the base

    def should_deploy(self, xyz):
        if self.radius is None:
            return False
        for loc in self.locations:
            if distance3D(xyz, loc) < self.radius:
                return False
        return True

    def on_sim_time_sec(self, data):
        if self.start_time is None:
            self.start_time = data
            if self.step_time is not None:
                self.next_deploy_time = self.start_time + self.step_time
        if self.next_deploy_time is not None and data > self.next_deploy_time:
            self.next_deploy_time += self.step_time
            self.publish('deploy', [])  # ROS std_msg/Empty expects empty list as input

    def on_pose3d(self, data):
        xyz, quat = data
        if self.should_deploy(xyz):
            self.locations.append(xyz)
            self.publish('deploy', [])
            self.publish('location', xyz)

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))

# vim: expandtab sw=4 ts=4
