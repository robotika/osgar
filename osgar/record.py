"""
  Recorder for given nodes configuration
"""

import argparse
import sys
import os
import signal
import threading

from osgar.logger import LogWriter
from osgar.lib.config import config_load, get_class_by_name
from osgar.bus import Bus


class Recorder:
    def __init__(self, config, logger):
        logger.write(0, bytes(str(config), 'ascii'))
        self.stop_requested = threading.Event()
        self.modules = {}

        bus = Bus(logger)
        for module_name, module_config in config['modules'].items():
            klass = get_class_by_name(module_config['driver'])
            self.modules[module_name] = klass(module_config['init'], bus=bus.handle(module_name))

        for sender, receiver in config['links']:
            bus.connect(sender, receiver, self.modules)

        signal.signal(signal.SIGINT, self.request_stop)

    def __enter__(self):
        for module in self.modules.values():
            module.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.request_stop()
        for module in self.modules.values():
            module.join()

    def request_stop(self, signum=None, frame=None): # pylint: disable=unused-argument
        if self.stop_requested.is_set():
            return
        for module in self.modules.values():
            module.request_stop()
        self.stop_requested.set()


def record(config, log_prefix, duration_sec=None):
    with LogWriter(prefix=log_prefix, note=str(sys.argv)) as log:
        with Recorder(config=config['robot'], logger=log) as recorder:
            if 'app' in recorder.modules:
                app = recorder.modules['app']
                app.join()
            else:
                recorder.stop_requested.wait(duration_sec)


def main():
    parser = argparse.ArgumentParser(description='Record run on real HW with given configuration')
    parser.add_argument('config', nargs='+', help='configuration file')
    parser.add_argument('--note', help='add description')
    parser.add_argument('--duration', help='recording duration (sec), default infinite', type=float)
    parser.add_argument('--application', help='import string to application', default=None)
    args = parser.parse_args()

    prefix = os.path.basename(args.config[0]).split('.')[0] + '-'
    cfg = config_load(*args.config, application=args.application)
    record(cfg, log_prefix=prefix, duration_sec=args.duration)

if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
