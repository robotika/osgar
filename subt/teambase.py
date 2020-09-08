"""
  OSGAR Teambase for Virtual
"""
from osgar.node import Node


class Teambase(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("broadcast")

        self.robot_name = config.get('robot_name')
        self.finish_time = None  # infinite
        if self.robot_name is not None:
            self.finish_time = int(self.robot_name[1:].split('F')[0])  # T100 is accepted for example

    def on_sim_time_sec(self, data):
        # broadcast simulation time every second
        self.publish('broadcast', b'%d' % data)
        if self.finish_time is not None and data > self.finish_time:
            self.request_stop()

    def update(self):
        channel = super().update()

        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))


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
