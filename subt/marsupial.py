"""
  Dual robots - typically a UGV taking care of UAV
  https://en.wikipedia.org/wiki/Marsupial
"""
from osgar.node import Node
from subt.name_decoder import parse_robot_name


class Marsupial(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("detach")

        self.start_time = None  # unknown
        self.release_at = config.get('release_at')  # not defined yet
        self.drone_available = True

    def detach(self):
        self.publish('detach', [])  # ROS std_msg/Empty expects empty list as input
        self.drone_available = False

    def on_sim_time_sec(self, data):
        if self.start_time is None:
            self.start_time = data

        if self.release_at is None:
            return

        if self.start_time + self.release_at <= data and self.drone_available:
            self.detach()

    def on_origin(self, data):
        if self.release_at is None:
            robot_name = data[0].decode('ascii')
            print(robot_name)
            cmd_list = parse_robot_name(robot_name)
            if len(cmd_list) > 0:
                if cmd_list[0][0] == 'wait':
                    self.release_at = cmd_list[0][1]

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # not supported

# vim: expandtab sw=4 ts=4
