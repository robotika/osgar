"""
  Multi-Trace-Manager for Virtual SubT

"""
from datetime import timedelta

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.lib.serialize import serialize, deserialize

# Note:
#  Payload size is limited to 1500 bytes - CommsClient::SendTo()
class MultiTraceManager(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('robot_trace', 'trace_info')

    def on_robot_trace(self, data):
        pass

    def on_robot_xyz(self, data):
        pass

    def update(self):
        channel = super().update()  # define self.time
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # not supported
        return channel

    def draw(self):
        pass

# vim: expandtab sw=4 ts=4
