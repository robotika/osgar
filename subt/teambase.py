"""
  OSGAR Teambase for Virtual
"""
from ast import literal_eval

from osgar.node import Node


class Teambase(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register()

        self.robot_name = config.get('robot_name')
        self.start_time = None  # unknown
        self.finish_time = None  # infinite
        if self.robot_name is not None:
            self.finish_time = int(self.robot_name[1:])  # T100 is accepted for example
        self.robot_positions = {}
        self.debug_arr = []
        self.artifacts = []
        self.verbose = False

    def on_sim_time_sec(self, data):
        if self.start_time is None:
            self.start_time = data
        if self.finish_time is not None and data - self.start_time > self.finish_time:
            self.request_stop()

    def on_robot_xyz(self, data):
        name, arr = data
        self.robot_positions[name] = arr
        if self.verbose:
            self.debug_arr.append((name, arr))

    def Xon_radio(self, data):
        src, packet = data
        name = src.decode('ascii')
        if packet.startswith(b'['):
            arr = literal_eval(packet.decode('ascii'))
            if len(arr) == 3:
                # position
                self.robot_positions[name] = arr
                if self.verbose:
                    self.debug_arr.append((name, arr))
            elif len(arr) == 4:
                # artifact
                if arr not in self.artifacts:
                    print(self.time, 'received:', arr)
                    self.artifacts.append(arr)
            else:
                assert False, arr  # unexpected size/type

    def update(self):
        channel = super().update()

        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))

    def draw(self):
        import matplotlib.pyplot as plt
        robot_ids = sorted(list(set([a[0] for a in self.debug_arr])))
        print('Robot IDs', robot_ids)

        for robot_id in robot_ids:
            x = [a[1][0] / 1000.0 for a in self.debug_arr if a[0] == robot_id]
            y = [a[1][1] / 1000.0 for a in self.debug_arr if a[0] == robot_id]
            line = plt.plot(x, y, '-o', linewidth=2, label=robot_id)

        artf_types = set([artf[0] for artf in self.artifacts])
        for artf_type in artf_types:
            pos = [pos for artf, pos, src, scored in self.artifacts if artf == artf_type]
            arr_x = [x/1000.0 for x, y, z in pos]
            arr_y = [y/1000.0 for x, y, z in pos]
            plt.plot(arr_x, arr_y, 'x', linewidth=2, label=artf_type)

        #    plt.xlabel('time (s)')
        plt.axes().set_aspect('equal', 'datalim')
        plt.legend()
        plt.show()


def main():
    import argparse
    import os
    from osgar.lib.config import config_load
    from osgar.record import record

    parser = argparse.ArgumentParser(description='SubT Teambase')
    parser.add_argument('config', nargs='+', help='configuration file')
    parser.add_argument('--note', help='add description')
    parser.add_argument('--log', nargs='?', help='record log filename')
    parser.add_argument('--robot-name', required=True, help='T<simulated seconds>F* name')
    args = parser.parse_args()

    cfg = config_load(*args.config)
    cfg['robot']['modules']['app']['init']['robot_name'] = args.robot_name
    prefix = os.path.basename(args.config[0]).split('.')[0] + '-'
    record(cfg, prefix, args.log)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
