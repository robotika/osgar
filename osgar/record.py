"""
  Recorder for given nodes configuration
"""

import argparse
import sys
import os
import time
from queue import Queue

from osgar.logger import LogWriter
from osgar.lib.config import load, get_class_by_name
from osgar.bus import BusHandler


class Recorder:
    def __init__(self, config, logger, application=None):
        self.modules = {}

        que = {}
        for module_name, module_config in config['modules'].items():
            out = {}
            for output_type in module_config['out']:
                out[output_type] = []
            bus = BusHandler(logger, out=out, name=module_name)
            que[module_name] = bus.queue

            module_class = module_config['driver']
            if module_class == 'application':
                assert application is not None  # external application required
                module = application(module_config['init'], bus=bus)
            else:
                module = get_class_by_name(module_class)(module_config['init'], bus=bus)

            self.modules[module_name] = module

        for from_module, to_module in config['links']:
            (in_driver, in_name), (out_driver, out_name) = from_module.split('.'), to_module.split('.')
            self.modules[in_driver].bus.out[in_name].append((self.modules[out_driver].bus.queue, out_name))

    def start(self):
        for module in self.modules.values():
            module.start()

    def update(self):
        pass

    def finish(self):
        for module in self.modules.values():
            module.request_stop()
        for module in self.modules.values():
            module.join()


def record(config_filename, duration_sec, log_prefix, note=''):
    log = LogWriter(prefix=log_prefix, note=str(sys.argv))
    config = load(config_filename)
    log.write(0, bytes(str(config), 'ascii'))  # write configuration
    recorder = Recorder(config=config['robot'], logger=log)
    recorder.start()
    time.sleep(duration_sec)
    recorder.finish()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Record run on real HW with given configuration')
    parser.add_argument('config', help='configuration file')
    parser.add_argument('--note', help='add description')
    parser.add_argument('--duration', help='recording duration (sec)',
                        type=float, default=3.0)
    args = parser.parse_args()

    prefix = os.path.basename(args.config).split('.')[0] + '-'
    record(args.config, args.duration, log_prefix=prefix, note=str(sys.argv))

# vim: expandtab sw=4 ts=4
