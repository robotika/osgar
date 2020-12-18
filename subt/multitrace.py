"""
  Multi-Trace-Manager for Virtual SubT

"""
from datetime import timedelta

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.lib.serialize import serialize, deserialize

# Note:
#  Payload size is limited to 1500 bytes - CommsClient::SendTo()

def get_intervals(seq):
    ret = []
    if len(seq) == 0:
        return ret
    prev = seq[0]
    start = prev
    for i in seq[1:]:
        if prev + 1 != i:
            ret.append([start, prev])
            start = i
        prev = i
    ret.append([start, prev])
    return ret


class MultiTraceManager(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('robot_trace', 'trace_info')
        self.traces = {}

    def publish_trace_info(self):
        info = {}
        for name in sorted(self.traces.keys()):
            info[name] = get_intervals([p[0] for p in self.traces[name]])
        self.publish('trace_info', info)

    def on_robot_trace(self, data):
        for name, positions in data.items():
            if name in self.traces:
                for p in positions:
                    if p not in self.traces[name]:
                        self.traces[name].append(p)
                self.traces[name] = sorted(self.traces[name])
            else:
                self.traces[name] = positions

    def on_robot_xyz(self, data):
        name, position = data
        if name not in self.traces:
            self.traces[name] = []
        self.traces[name].append(position)
        self.publish_trace_info()

    def on_trace_info(self, data):
        update = {}
        for name, positions in self.traces.items():
            if name in data:
                part = []
                for t, xyz in positions:
                    for from_index, to_index in data[name]:
                        if from_index <= t <= to_index:
                            break
                    else:
                        part.append([t, xyz])
                if len(part) > 0:
                    update[name] = part
            else:
                update[name] = positions
        if len(update) > 0:
            self.publish('robot_trace', update)

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