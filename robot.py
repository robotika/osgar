"""
  Robot - container for drivers
"""

import argparse
import sys
from queue import Queue

from lib.logger import LogWriter, LogReader
from lib.config import Config
from drivers import all_drivers
from drivers.bus import BusHandler


class Robot:
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
                module = all_drivers[module_class](module_config['init'], bus=bus)
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


if __name__ == "__main__":
    import time

    parser = argparse.ArgumentParser(description='Test robot configuration')
    subparsers = parser.add_subparsers(help='sub-command help', dest='command')
    subparsers.required = True
    parser_run = subparsers.add_parser('run', help='run on real HW')
    parser_run.add_argument('config', help='configuration file')
    parser_run.add_argument('--note', help='add description')

    parser_replay = subparsers.add_parser('replay', help='replay from logfile')
    parser_replay.add_argument('logfile', help='recorded log file')
    args = parser.parse_args()

    if args.command == 'replay':
        pass  # TODO
    elif args.command == 'run':
        log = LogWriter(prefix='robot-test-', note=str(sys.argv))
        config = Config.load(args.config)
        log.write(0, bytes(str(config.data), 'ascii'))  # write configuration
        robot = Robot(config=config.data['robot'], logger=log)
        robot.start()
        time.sleep(3.0)
        robot.finish()
    else:
        assert False, args.command  # unsupported command

# vim: expandtab sw=4 ts=4
