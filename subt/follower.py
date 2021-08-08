"""
  SubT radio follower

Follow trace of already running robot (with shortcuts)
"""

from osgar.node import Node


class RadioFollower(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('waypoints')
        self.robot_name_prefix = None

    def on_robot_name(self, data):
        name = data.decode('ascii')
        if 'X' in name and 'XM' not in name:  # not mapping
            self.robot_name_prefix = name.split('X')[1]

    def on_robot_xyz(self, data):
        name, position = data


    def update(self):
        channel = super().update()  # define self.time
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # not supported
        return channel


# vim: expandtab sw=4 ts=4
