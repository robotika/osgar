"""
  Recorder for given nodes configuration
"""

import argparse
import sys
import os
import math

import trio

from osgar.logger import LogWriter
from osgar.lib.config import load, get_class_by_name
import osgar.bus


async def build_graph(nursery, config, logger, application):
    # stop_requested = trio.Event() # TODO: see end of function
    buses = {}
    ret = None

    for module_name, module_config in config['modules'].items():
        buses[module_name] = await osgar.bus.BusHandler(logger, name=module_name, out=module_config['out'])

    for from_module, to_module in config['links']:
        (from_driver, from_name), (to_driver, to_name) = from_module.split('.'), to_module.split('.')
        osgar.bus.connect(buses[from_driver], from_name, buses[to_driver], to_name)

    for module_name, module_config in config['modules'].items():
        module_class = module_config['driver']
        if module_class == 'application':
            assert application is not None  # external application required
            ret = nursery.start_soon(application, nursery, module_config['init'], buses[module_name])
        else:
            nursery.start_soon(get_class_by_name(module_class), nursery, module_config['init'], buses[module_name])

    # signal.signal(signal.SIGINT, self.request_stop) # TODO
    return ret


async def record(config, log_prefix, duration_sec=math.inf, application=None):
    async with LogWriter(prefix=log_prefix, note=str(sys.argv)) as log:
        await log.write(0, bytes(str(config), 'ascii'))  # write configuration
        with trio.move_on_after(duration_sec):
            async with trio.open_nursery() as nursery:
                app = await build_graph(nursery, config['robot'], log, application)
                if app is not None:
                    await app
                    nursery.cancel_scope.cancel()


def main():
    parser = argparse.ArgumentParser(description='Record run on real HW with given configuration')
    parser.add_argument('config', nargs='+', help='configuration file')
    parser.add_argument('--note', help='add description')
    parser.add_argument('--duration', help='recording duration (sec), default infinite', type=float, default=math.inf)
    args = parser.parse_args()
    config = load(*args.config)
    prefix = os.path.basename(args.config[0]).split('.')[0] + '-'
    trio.run(record, config, prefix, args.duration)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
