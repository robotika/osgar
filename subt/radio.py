"""
  Radio for Virtual SubT

"""
from datetime import timedelta
from ast import literal_eval

from osgar.node import Node
from osgar.bus import BusShutdownException


def Xdraw_lora_positions(arr):
    """
    Draw positions of tripples:
        (time, ID, (x. y, heading))
    """
    import matplotlib.pyplot as plt
    t = [a[0] for a in arr]
    robot_ids = sorted(list(set([a[1] for a in arr])))
    print('Robot IDs', robot_ids)

    for robot_id in robot_ids:
        x = [a[2][0]/1000.0 for a in arr if a[1] == robot_id]
        y = [a[2][1]/1000.0 for a in arr if a[1] == robot_id]
        line = plt.plot(x, y, '-o', linewidth=2, label='Robot #%d' % robot_id)

#    plt.xlabel('time (s)')
    plt.axes().set_aspect('equal', 'datalim')
    plt.legend()
    plt.show()


class Radio(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('radio', 'artf_xyz')
        self.min_transmit_dt = timedelta(seconds=10)  # TODO config?
        self.last_transmit = None
        self.recent_packets = []
        self.verbose = config.get('verbose', False)
        self.debug_robot_poses = []

    def send_data(self, data):
        self.last_transmit = self.publish('radio', data + b'\n')
        return self.last_transmit

    def on_radio(self, data):
        src, packet = data
        name = src.decode('ascii')
        if packet.startswith(b'['):
            arr = literal_eval(packet.decode('ascii'))
            if len(arr) == 3:
                # position
                pass
            elif len(arr) == 4:
                # artifact
                self.publish('artf_xyz', [arr])  # publish also standard "list" of detected artifacts
            else:
                assert False, arr  # unexpected size/type

    def update(self):
        channel = super().update()  # define self.time
        if channel == 'radio':
            self.on_radio(self.radio)
        elif channel == 'pose2d':
            if self.last_transmit is None or self.time > self.last_transmit + self.min_transmit_dt:
                self.send_data(bytes(str(self.pose2d), encoding='ascii'))
        elif channel == 'artf':
            # send data as they are, ignore transmit time, ignore transmit failure
            for artf_item in self.artf:
                self.send_data(bytes(str(artf_item), encoding='ascii'))
        else:
            assert False, channel  # not supported
        return channel

    def draw(self):
        """
        Debug Draw
        """
        draw_lora_positions(self.debug_robot_poses)


# vim: expandtab sw=4 ts=4
