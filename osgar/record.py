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
        self.stop_requested = threading.Event()
        self.modules = {}

        bus = Bus(logger)
        for module_name, module_config in config['modules'].items():
            klass = get_class_by_name(module_config['driver'])
            self.modules[module_name] = klass(module_config['init'], bus=bus.handle(module_name))

        for sender, receiver in config['links']:
            bus.connect(sender, receiver, self.modules)

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
    if isinstance(config_filename, str):
        config_filename = [config_filename]
    config = config_load(*config_filename, application=application)
    with LogWriter(prefix=log_prefix, note=str(sys.argv)) as log:
        try:
            log.write(0, bytes(str(config), 'ascii'))  # write configuration
            recorder = Recorder(config=config['robot'], logger=log)
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
