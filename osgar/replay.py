"""
  Replay driver modules from log file
"""

import argparse
import sys
import math
from ast import literal_eval
from datetime import timedelta
from queue import Queue

import numpy as np

from osgar.logger import LogReader
from osgar.lib.config import load as config_load
from osgar.bus import LogBusHandler, LogBusHandlerInputsOnly
from osgar.drivers import all_drivers


def replay(args, application=None):
    log = LogReader(args.logfile)
    print(next(log.read_gen(0))[-1])  # old arguments
    config_str = next(log.read_gen(0))[-1]
    config = literal_eval(config_str.decode('ascii'))
    if args.config is not None:
        config = config_load(*args.config)

    names = []
    for __, __, line in log.read_gen(0):
        d = literal_eval(line.decode('ascii'))
        if 'names' in d:
            names = d['names']
    print(names)
    
    module = args.module
    assert module in config['robot']['modules'], (module, config['robot']['modules'])
    module_config = config['robot']['modules'][module]

    input_names = module_config['in']
    output_names = module_config['out']
    print(input_names, output_names)

    inputs = {}
    for edge_from, edge_to in config['robot']['links']:
        if edge_to.split('.')[0] == module:
            inputs[1 + names.index(edge_from)] = edge_to.split('.')[1]
    print(inputs)

    outputs = dict([(1 + names.index('.'.join([module, name])), name) for name in output_names])
    print(outputs)

    log = LogReader(args.logfile)  # start reading log from the beginning again
    if args.force:
        bus = LogBusHandlerInputsOnly(log, inputs=inputs)
    else:
        bus = LogBusHandler(log, inputs=inputs, outputs=outputs)

    driver_name = module_config['driver']
    if driver_name == 'application':
        assert application is not None
        module_class = application(module_config['init'], bus=bus)
    else:
        module_class = all_drivers[driver_name](module_config['init'], bus=bus)
    return module_class


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Replay module from log')
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--force', '-F', dest='force', action='store_true', help='force replay even for failing output asserts')
    parser.add_argument('--config', nargs='+', help='force alternative configuration file')
    parser.add_argument('--module', help='module name for analysis', required=True)  # TODO default "all"
    args = parser.parse_args()

    module_class = replay(args)

    module_class.start()
    # now wait until the module is alive
    module_class.join()

# vim: expandtab sw=4 ts=4
