"""
  Keep given buffer of RGBD images until crash happens (detected in accelerometers)
"""
import collections

from osgar.node import Node
from subt.trace import distance3D


class CrashReport(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("crash_rgbd")
        size = config.get('size', 100)  # keep N last records
        self.acc_limit = config.get('acc_limit', 100.0)  # i.e. 10g
        self.buf = collections.deque(maxlen=size)

    def on_rgbd(self, data):
        self.buf.append(data)

    def on_acc(self, data):
        vec = [x/1000.0 for x in data]
        value = abs(distance3D(vec, [0, 0, 0]) - 9.81)
        if value >= self.acc_limit:
            for rec in self.buf:
                self.publish('crash_rgbd', rec)
            self.buf.clear()

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # unsupported channel
        return channel

# vim: expandtab sw=4 ts=4
