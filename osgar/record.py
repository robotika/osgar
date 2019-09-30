"""
  Recorder for given nodes configuration
"""

import argparse
import sys
import os
import time
import signal
import threading

from osgar.logger import LogWriter
from osgar.lib.config import load, get_class_by_name
from osgar.bus import BusHandler


class Recorder:
    def __init__(self, config, logger, application=None):
        self.stop_requested = threading.Event()
        self.modules = {}

        buses = {}
        for module_name, module_config in config['modules'].items():
            out, slots = {}, {}
            for output_type in module_config['out']:
                out[output_type] = []
                slots[output_type] = []
            bus = BusHandler(logger, out=out, slots=slots, name=module_name)
            buses[module_name] = bus

            module_class = module_config['driver']
            if module_class == 'application':
                assert application is not None  # external application required
                module = application(module_config['init'], bus=bus)
            else:
                module = get_class_by_name(module_class)(module_config['init'], bus=bus)

            self.modules[module_name] = module

        for from_module, to_module in config['links']:
            (from_driver, from_name), (to_driver, to_name) = from_module.split('.'), to_module.split('.')
            if to_name.startswith('slot_'):
                buses[from_driver].slots[from_name].append(getattr(self.modules[to_driver], to_name))
            else:
                buses[from_driver].out[from_name].append((buses[to_driver].queue, to_name))

        signal.signal(signal.SIGINT, self.request_stop)

    def start(self):
        for module in self.modules.values():
            module.start()

    def update(self):
        pass

    def finish(self):
        self.request_stop()
        self.join()

    def request_stop(self, sig=None, frame=None):
        if self.stop_requested.is_set():
            return
        for module in self.modules.values():
            module.request_stop()
        self.stop_requested.set()

    def join(self):
        for module in self.modules.values():
            module.join()



def record(config_filename, log_prefix, duration_sec=None, application=None):
    if type(config_filename) == str:
        config = load(config_filename)
    else:
        config = load(*config_filename)
    with LogWriter(prefix=log_prefix, note=str(sys.argv)) as log:
        try:
            log.write(0, bytes(str(config), 'ascii'))  # write configuration
            recorder = Recorder(config=config['robot'], logger=log, application=application)
            recorder.start()
            if application is not None:
                app = recorder.modules['app']  # TODO nicer reference
                app.join()  # wait for application termination
            else:
                recorder.stop_requested.wait(duration_sec)
        finally:
            recorder.finish()


def main():
    parser = argparse.ArgumentParser(description='Record run on real HW with given configuration')
    parser.add_argument('config', nargs='+', help='configuration file')
    parser.add_argument('--note', help='add description')
    parser.add_argument('--duration', help='recording duration (sec), default infinite', type=float)
    args = parser.parse_args()

    prefix = os.path.basename(args.config[0]).split('.')[0] + '-'
    record(args.config, log_prefix=prefix, duration_sec=args.duration)

if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
