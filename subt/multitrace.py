"""
  Multi-Trace-Manager for Virtual SubT

The MTM has input "robot_xyz" and publishes overview info of known positions of other robots "trace_info".
This "trace_info" contains list of time intervals for each known robot. The response is then "robot_trace"
which is limited in size and contains missing bits of received "trace_info".
"""
from datetime import timedelta

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.lib.serialize import serialize, deserialize
from subt.trace import distance3D

# Note:
#  Payload size is limited to 1500 bytes - CommsClient::SendTo()

SIZE_LIMIT = 1000  # just guess, not to reach the limit
CUT_NUM = 40  # limit number of positions to fit 1500 bytes limit (50 positions = 1620 bytes)


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


def find_shortcut(position, trace, radius=2.0):
    """
    For given position [t, xyz] find in the trace (list of [t, xyz])
    the first index when distance is shorter than given radius.
    """
    last_xyz = position[1]  # ignore sim_time_sec
    for i, (t, xyz) in enumerate(trace):
        if distance3D(last_xyz, xyz) < radius:
            break
    return i


class MultiTraceManager(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('robot_trace', 'trace_info', 'response', 'time_to_signal')
        self.traces = {}
        self.robot_name = None  # my name (unknown on start)
        self.signal_times = []  # times received from Teambase
        self.time_to_signal = []
        self.shortcut = []
        self.verbose = False

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
        if self.verbose:
            print('robot_xyz', data)
        name, position = data
        if name not in self.traces:
            self.traces[name] = []
        self.traces[name].append(position)
        self.publish_trace_info()
        if name == self.robot_name:
            index = find_shortcut(position, self.traces[name])
            print('index', index, len(self.traces[name]), len(self.time_to_signal))
            self.shortcut.append(index)
            if len(self.time_to_signal) > 0:
                if index < len(self.time_to_signal):
                    tts = 1 + min(self.time_to_signal[-1], self.time_to_signal[index])
                else:
                    tts = 1 + self.time_to_signal[-1]
            else:
                tts = 0
            self.time_to_signal.append(tts)

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
            if len(str(update)) < SIZE_LIMIT:
                self.publish('robot_trace', update)
            else:
                # cut first part for now
                for name in update:
                    self.publish('robot_trace', {name: update[name][:CUT_NUM]})

    def on_query(self, data):
        name, from_sec, to_sec = data
        self.publish('response', {name: self.traces.get(name, [])})

    def on_robot_name(self, data):
        if self.verbose:
            print('Robot name:', data)
        self.robot_name = data.decode('ascii')
        if self.robot_name in self.traces:
            # prefill array of old position to match the size of known trace
            size = len(self.traces[self.robot_name])
            self.shortcut = [0] * size
            self.time_to_signal = [0] * size

    def on_teambase_sec(self, data):
        if self.verbose:
            print('Teambase:', data)
        self.signal_times.append(data)

    def on_sim_time_sec(self, data):
        if self.verbose:
            print('Sim Time:', data)
        if len(self.signal_times) > 0:
            # TODO more complex algo
            tts = data - self.signal_times[-1]
        else:
            tts = 0  # starting area, but did not receive message from Teambase yet
        self.publish('time_to_signal', tts)

    def update(self):
        channel = super().update()  # define self.time
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # not supported
        return channel

    def draw(self):
        import matplotlib.pyplot as plt
        robot_ids = sorted(self.traces)
        print('Robot IDs', robot_ids)

        for robot_id in robot_ids:
            x = [a[1][0] for a in self.traces[robot_id]]
            y = [a[1][1] for a in self.traces[robot_id]]
            line = plt.plot(x, y, '-o', linewidth=2, label=robot_id)

        plt.axes().set_aspect('equal', 'datalim')
        plt.legend()
        plt.show()

    def draw3d(self):
        """
        Alternative 3D plot not available via --draw. At the moment the view is very limited
        but it is an example how 3D points can be shown/analyzed.
        """
        # https://matplotlib.org/mpl_toolkits/mplot3d/tutorial.html
        import matplotlib as mpl
        from mpl_toolkits.mplot3d import Axes3D
        import matplotlib.pyplot as plt

        robot_ids = sorted(self.traces)
        print('Robot IDs', robot_ids)

        mpl.rcParams['legend.fontsize'] = 10
        fig = plt.figure()
        ax = fig.gca(projection='3d')

        for robot_id in robot_ids:
            x = [a[1][0] for a in self.traces[robot_id]]
            y = [a[1][1] for a in self.traces[robot_id]]
            z = [a[1][2] for a in self.traces[robot_id]]
            ax.plot(x, y, z, 'o-', label=robot_id)
            ax.legend()

        plt.show()

# vim: expandtab sw=4 ts=4
