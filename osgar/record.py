"""
  Recorder for given nodes configuration
"""

import argparse
import sys
import os
import signal
import threading
import logging

from osgar.logger import LogWriter
from osgar.lib.config import config_load, get_class_by_name
from osgar.bus import Bus

g_logger = logging.getLogger(__name__)

class Recorder:
    def __init__(self, config, logger):
        self.stop_requested = threading.Event()
        self.sigint_received = False
        self.modules = {}

        self.bus = Bus(logger)
        for module_name, module_config in config['modules'].items():
            klass = get_class_by_name(module_config['driver'])
            self.modules[module_name] = klass(module_config['init'], bus=self.bus.handle(module_name))

        for sender, receiver in config['links']:
            self.bus.connect(sender, receiver, self.modules)

        signal.signal(signal.SIGINT, self.request_stop)

    def __enter__(self):
        for module in self.modules.values():
            module.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.request_stop()
        for module in self.modules.values():
            module.join(1)
        for name, max_delay, timestamp in self.bus.delays():
            g_logger.error(f"{name:>12}: maximum delay of {max_delay} at {timestamp}")
        for t in threading.enumerate():
            if t != threading.current_thread():
                g_logger.error(f'thread {repr(t.name)} still running!')
                g_logger.error(f'    class: {t.__class__.__module__}.{t.__class__.__name__}')
                g_logger.error(f'    target: {t._target}')
        if self.sigint_received:
            g_logger.info("committing suicide by SIGINT")
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            os.kill(os.getpid(), signal.SIGINT)

    def request_stop(self, signum=None, frame=None): # pylint: disable=unused-argument
        if signum == signal.SIGINT:
            self.sigint_received = True
        if self.stop_requested.is_set():
            return
        for module in self.modules.values():
            module.request_stop()
        self.stop_requested.set()


def record(config, log_prefix, log_filename=None, duration_sec=None):
    with LogWriter(prefix=log_prefix, filename=log_filename, note=str(sys.argv)) as log:
        log.write(0, bytes(str(config), 'ascii'))
        g_logger.info(log.filename)
        with Recorder(config=config['robot'], logger=log) as recorder:
            if 'app' in recorder.modules:
                app = recorder.modules['app']
                app.join(duration_sec)
            else:
                recorder.stop_requested.wait(duration_sec)


def main():
    import logging
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s %(name)-16s %(levelname)-8s %(message)s',
    )
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
