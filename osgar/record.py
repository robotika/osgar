"""
  Recorder for given nodes configuration
"""

import argparse
import sys
import os
import signal
import threading
import logging
import math
import time

from osgar.logger import LogWriter
from osgar.lib.config import config_load, get_class_by_name
from osgar.bus import Bus

g_logger = logging.getLogger(__name__)

class Recorder:
    def __init__(self, config, logger):
        self.stop_requested = threading.Event()
        self.sigint_received = False
        self.modules = {}
        self.app_waiter = None
        self.state = 'RUNNING'
        self.shutdown_reason = 'requested'
        self.shutdown_protocols = {}
        self.shutdown_timeout = config.get('shutdown_timeout', 5.0)
        if (not isinstance(self.shutdown_timeout, (int, float))
                or not math.isfinite(self.shutdown_timeout) or self.shutdown_timeout <= 0):
            raise ValueError('shutdown_timeout must be a positive finite number of seconds')
        self.shutdown_groups = {}
        for name, module_config in config['modules'].items():
            if name == '_system':
                raise ValueError('_system is reserved for lifecycle messages')
            protocol = module_config.get('shutdown_protocol', 'legacy')
            if protocol not in ('legacy', 'messages'):
                raise ValueError(f'Unknown shutdown_protocol for {name}: {protocol!r}')
            self.shutdown_protocols[name] = protocol
            group = module_config.get('shutdown_group', 'controller')
            if group not in ('controller', 'transport'):
                raise ValueError(f'Unknown shutdown_group for {name}: {group!r}')
            self.shutdown_groups[name] = group

        self.bus = Bus(logger)
        env = config.get('env')
        for module_name, module_config in config['modules'].items():
            klass = get_class_by_name(module_config['driver'])
            module_config_init = module_config.get('init', {})
            if env is not None:
                assert 'env' not in module_config_init, module_config_init
                module_config_init['env'] = env.copy()
            bus_handle = self.bus.handle(module_name)
            if 'out' in module_config:
                bus_handle.config_out = module_config['out']
            self.modules[module_name] = klass(module_config_init, bus=bus_handle)

        for sender, receiver in config['links']:
            if (receiver.endswith('.shutdown')
                    and self.shutdown_protocols.get(receiver.split('.')[0]) == 'messages'):
                raise ValueError('shutdown is a reserved system input')
            self.bus.connect(sender, receiver, self.modules)

        for name, protocol in self.shutdown_protocols.items():
            if protocol == 'messages':
                self.bus.enable_shutdown(name)

        signal.signal(signal.SIGINT, self.request_stop)
        g_logger.info("SIGINT handler installed")

    def __enter__(self):
        for module in self.modules.values():
            module.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is not None:
            self.shutdown_reason = 'exception'
        self.request_stop()
        self._stop_group('controller', 'STOPPING')
        self._stop_group('transport', 'DRAINING')
        for name, max_delay, timestamp in self.bus.delays():
            g_logger.error(f"{name:>12}: maximum delay of {max_delay} at {timestamp}")
        for t in threading.enumerate():
            if t != threading.current_thread():
                g_logger.error(f'thread {repr(t.name)} still running!')
                g_logger.error(f'    class: {t.__class__.__module__}.{t.__class__.__name__}')
                g_logger.error(f'    target: {getattr(t, "_target", None)}')
        self.bus.close()  # seal handles before closing the log, including after a timeout
        self.bus.logger.close()  # save not flushed data
        self.state = 'CLOSED'
        if self.sigint_received:
            g_logger.info("committing suicide by SIGINT")
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            os.kill(os.getpid(), signal.SIGINT)

    def request_stop(self, signum=None, frame=None): # pylint: disable=unused-argument
        """Wake the recorder; teardown runs outside the signal handler."""
        if signum == signal.SIGINT:
            self.sigint_received = True
            self.shutdown_reason = 'sigint'
        self.stop_requested.set()
        self.bus.stop_requested.set()

    def _stop_group(self, group, state):
        self.state = state
        g_logger.info('Shutdown phase: %s', state)
        deadline = time.monotonic() + self.shutdown_timeout
        workers = {}
        failures = {}

        def stop_and_join(name, module):
            try:
                if self.shutdown_protocols.get(name) == 'messages':
                    remaining = max(0, deadline - time.monotonic())
                    self.bus.send_shutdown(name, {'phase': 'prepare', 'reason': self.shutdown_reason,
                                                  'timeout': remaining, 'deadline': time.time() + remaining})
                    if not self.bus.handle(name).shutdown_ready_event.wait(max(0, deadline - time.monotonic())):
                        raise TimeoutError('shutdown readiness not received')
                    self.bus.send_shutdown(name, {'phase': 'close', 'reason': self.shutdown_reason,
                                                  'timeout': max(0, deadline - time.monotonic()),
                                                  'deadline': time.time() + max(0, deadline - time.monotonic())})
                else:
                    module.request_stop()
                module.join()
                if name == 'app' and self.app_waiter is not None:
                    self.app_waiter.join()
            except Exception as error:
                failures[name] = error

        for name, module in self.modules.items():
            if self.shutdown_groups.get(name, 'controller') == group:
                worker = threading.Thread(target=stop_and_join, args=(name, module),
                                          name=f'stop-{name}', daemon=True)
                workers[name] = worker
                worker.start()
        for name, worker in workers.items():
            worker.join(max(0, deadline - time.monotonic()))
            if worker.is_alive():
                g_logger.error('Shutdown timeout in %s: %s', state, name)
                self.bus.handle(name).shutdown()
            elif name in failures:
                g_logger.error('Shutdown failed in %s: %s: %s', state, name, failures[name])
                self.bus.handle(name).shutdown()


def record(config, log_prefix, log_filename=None, duration_sec=None):
    with LogWriter(prefix=log_prefix, filename=log_filename, note=str(sys.argv)) as log:
        # fill system variables
        if 'env' in config['robot']:
            for k in list(config['robot']['env'].keys()):
                config['robot']['env'][k] = os.getenv(k)
        log.write(0, bytes(str(config), 'ascii'))
        g_logger.info(log.filename)
        with Recorder(config=config['robot'], logger=log) as recorder:
            if 'app' in recorder.modules:
                app = recorder.modules['app']
                def wait_for_app():
                    app.join()
                    recorder.bus.stop_requested.set()
                app_waiter = threading.Thread(target=wait_for_app, name='app-waiter', daemon=True)
                recorder.app_waiter = app_waiter
                app_waiter.start()
            if not recorder.bus.stop_requested.wait(duration_sec):
                recorder.shutdown_reason = 'duration'


def main(record=record):
    import logging
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s %(name)-16s %(levelname)-8s %(message)s',
    )
    parser = argparse.ArgumentParser(description='Record run on real HW with given configuration')
    parser.add_argument('config', nargs='+', help='configuration file')
    parser.add_argument('--note', help='add description')
    parser.add_argument('--duration', help='seconds before graceful shutdown begins, default infinite', type=float)
    parser.add_argument('--log', help='force record log filename')
    parser.add_argument('--application', help='import string to application', default=None)
    parser.add_argument('--params', nargs='+',
                        help='optional list of configuration parameters like app.max_speed=0.1 app.dist=-2.0. '
                             'For string parameters use double quotes e.g.: app.logfile=\'"name.log"\'')
    parser.add_argument('--without', nargs='+',
                        help='list of modules which should not be loaded/initialized/started')
    args = parser.parse_args()


    prefix = os.path.basename(args.config[0]).split('.')[0] + '-'
    cfg = config_load(*args.config, application=args.application, params=args.params, without=args.without)
    record(cfg, log_prefix=prefix, duration_sec=args.duration, log_filename=args.log)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
