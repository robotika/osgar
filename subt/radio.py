"""
  Radio for Virtual SubT

"""
from datetime import timedelta

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.lib.serialize import serialize, deserialize

# Note:
#  Payload size is limited to 1500 bytes - CommsClient::SendTo()

def draw_positions(arr):
    """
    Draw positions of tripples:
        (time, ID, (x. y, heading))
    """
    import matplotlib.pyplot as plt
    t = [a[0] for a in arr]
    robot_ids = sorted(list(set([a[1] for a in arr])))
    print('Robot IDs', robot_ids)

    for robot_id in robot_ids:
        # received time, robot name, [sim_time, robot xyz position]
        x = [a[2][1][0] for a in arr if a[1] == robot_id]
        y = [a[2][1][1] for a in arr if a[1] == robot_id]
        line = plt.plot(x, y, '-o', linewidth=2, label=f'Robot {robot_id}')

#    plt.xlabel('time (s)')
    plt.axes().set_aspect('equal', 'datalim')
    plt.legend()
    plt.show()


class Radio(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('radio', 'artf_xyz', 'breadcrumb', 'robot_xyz', 'trace_info', 'robot_trace')
        self.min_transmit_dt = 0  # i.e. every sim_time_sec -> every simulated second
        self.last_transmit = None
        self.sim_time_sec = None
        self.recent_packets = []
        self.verbose = config.get('verbose', False)
        self.debug_robot_poses = []
        self.message_counter = 0

    def send_data(self, channel, data):
        raw = serialize([self.message_counter, channel, data])
        self.publish('radio', raw)
        self.message_counter += 1

    def on_radio(self, data):
        src, packet = data
        name = src.decode('ascii')
        __, channel, msg_data = deserialize(packet)
        if channel == 'artf':
            self.publish('artf_xyz', msg_data)  # topic rename - beware of limits 1500bytes!
        elif channel in ['breadcrumb', 'trace_info', 'robot_trace']:
            self.publish(channel, msg_data)
        elif channel == 'xyz':
            self.publish('robot_xyz', [name, msg_data])
            if self.verbose:
                self.debug_robot_poses.append((self.time, name, msg_data))

    def on_breadcrumb(self, data):
        self.send_data('breadcrumb', data)

    def on_pose3d(self, data):
        if self.sim_time_sec is None:
            return
        if self.last_transmit is None or self.sim_time_sec > self.last_transmit + self.min_transmit_dt:
            self.send_data('xyz', [self.sim_time_sec, data[0]])
            self.last_transmit = self.sim_time_sec

    def on_artf(self, data):
        self.send_data('artf', data)

    def on_trace_info(self, data):
        self.send_data('trace_info', data)

    def on_robot_trace(self, data):
        self.send_data('robot_trace', data)

    def on_sim_time_sec(self, data):
        self.sim_time_sec = data  # duplicate for super().update(), used just for easier unittesting
        self.send_data('sim_time_sec', data)

    def update(self):
        channel = super().update()  # define self.time
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # not supported
        return channel

    def draw(self):
        """
        Debug Draw
        """
        draw_positions(self.debug_robot_poses)


# vim: expandtab sw=4 ts=4
