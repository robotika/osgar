"""
  Replay driver modules from log file
"""

import argparse
import sys
import math
from ast import literal_eval
from datetime import timedelta
from queue import Queue

from osgar import logger
from osgar.logger import LogReader
from osgar.lib.config import load as config_load, get_class_by_name
from osgar.bus import LogBusHandler, LogBusHandlerInputsOnly


def replay(args, application=None):
    log = LogReader(args.logfile, only_stream_id=0)
    print("original args:", next(log)[-1])  # old arguments
    config_str = next(log)[-1]
    config = literal_eval(config_str.decode('ascii'))
    if args.config is not None:
        config = config_load(*args.config)

    names = logger.lookup_stream_names(args.logfile)
    print("stream names:")
    for name in names:
        print(" ", name)
    
    module = args.module
    assert module in config['robot']['modules'], (module, list(config['robot']['modules'].keys()))
    module_config = config['robot']['modules'][module]

    input_names = module_config['in']
    output_names = module_config['out']
    print("inputs:", input_names)
    print("outputs:", output_names)

    inputs = {}
    for edge_from, edge_to in config['robot']['links']:
        if edge_to.split('.')[0] == module:
            inputs[1 + names.index(edge_from)] = edge_to.split('.')[1]
    #print(inputs)

    outputs = dict([(1 + names.index('.'.join([module, name])), name) for name in output_names])
    #print(outputs)

    # start reading log from the beginning again
    if args.force:
        log = LogReader(args.logfile, only_stream_id=inputs.keys())
        bus = LogBusHandlerInputsOnly(log, inputs=inputs)
    else:
        streams = list(inputs.keys()) + list(outputs.keys())
        log = LogReader(args.logfile, only_stream_id=streams)
        bus = LogBusHandler(log, inputs=inputs, outputs=outputs)

    driver_name = module_config['driver']
    if driver_name == 'application':
        assert application is not None
        module_class = application
    else:
        module_class = get_class_by_name(driver_name)
    return module_class(module_config['init'], bus=bus)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Replay module from log')
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--force', '-F', dest='force', action='store_true', help='force replay even for failing output asserts')
    parser.add_argument('--config', nargs='+', help='force alternative configuration file')
    parser.add_argument('--module', help='module name for analysis', required=True)  # TODO default "all"
    parser.add_argument('--verbose', '-v', help="verbose mode", action='store_true')
    args = parser.parse_args()

    module_instance = replay(args)
    module_instance.verbose = args.verbose

    module_instance.start()
    # now wait until the module is alive
    module_instance.join()
    print("maximum delay:", module_instance.bus.max_delay)

# vim: expandtab sw=4 ts=4
