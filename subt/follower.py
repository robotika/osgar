"""
  SubT radio follower

Follow trace of already running robot (with shortcuts)
"""

from osgar.node import Node


class RadioFollower(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('waypoints', 'query_trace')
        self.robot_name_prefix = None
        self.robot_names = []
        self.leader_trace = []

    def get_leader_robot_name(self):
        if self.robot_name_prefix is None:
            return None
        for name in self.robot_names:
            if name.startswith(self.robot_name_prefix):
                return name
        return None

    def on_robot_name(self, data):
        name = data.decode('ascii')
        if 'X' in name and 'XM' not in name:  # not mapping
            self.robot_name_prefix = name.split('X')[1]

    def on_sim_time_sec(self, data):
        leader_name = self.get_leader_robot_name()
        if leader_name is not None:
            # query the whole trace from the very beginning
            self.publish('query_trace', [leader_name, 0, data])

    def on_robot_xyz(self, data):
        name, position_with_time = data
        if name not in self.robot_names:
            self.robot_names.append(name)

    def on_pose3d(self, data):
        pass

    def on_trace(self, data):
        pass

    def update(self):
        channel = super().update()  # define self.time
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # not supported
        return channel


# vim: expandtab sw=4 ts=4
