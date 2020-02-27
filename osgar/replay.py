"""
  Replay driver modules from log file
"""

import argparse
import logging
from ast import literal_eval

from osgar import logger
from osgar.logger import LogReader
from osgar.lib.config import config_load, get_class_by_name
from osgar.bus import LogBusHandler, LogBusHandlerInputsOnly


def replay(args, application=None):
    log = LogReader(args.logfile, only_stream_id=0)
    print("original args:", next(log)[-1])  # old arguments
    config_str = next(log)[-1]
    config = literal_eval(config_str.decode('ascii'))
    if args.config is not None:
        config = config_load(*args.config, application=application)

    names = logger.lookup_stream_names(args.logfile)
    if args.debug:
        print("streams:")
        for i, name in enumerate(names):
            print(f" {i+1:2d} {name}")
    
    module = args.module
    assert module in config['robot']['modules'], (module, list(config['robot']['modules'].keys()))
    module_config = config['robot']['modules'][module]

    inputs = {}
    for edge_from, edge_to in config['robot']['links']:
        if edge_to.split('.')[0] == module:
            if edge_from not in names:
                logging.warning('Missing name: %s' % edge_from)
                names.append(edge_from)
            inputs[1 + names.index(edge_from)] = edge_to.split('.')[1]

    outputs = {i + 1: out.split('.')[1] for i, out in enumerate(names) if out.startswith(f"{module}.")}
    if args.debug:
        print("inputs:")
        for i, name in sorted(inputs.items()):
            print(f" {i:2d} {name}")
        print("outputs:")
        for i, name in sorted(outputs.items()):
            print(f" {i:2d} {name}")

    if args.force:
        reader = LogReader(args.logfile, only_stream_id=inputs.keys())
        bus = LogBusHandlerInputsOnly(reader, inputs=inputs)
    else:
        streams = list(inputs.keys()) + list(outputs.keys())
        reader = LogReader(args.logfile, only_stream_id=streams)
        bus = LogBusHandler(reader, inputs, outputs)

    driver_name = module_config['driver']
    module_class = get_class_by_name(driver_name)
    module_instance = module_class(module_config['init'], bus=bus)

    bus.node = module_instance # needed for slots
    return module_instance


def main():
    parser = argparse.ArgumentParser(description='Replay module from log')
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--force', '-F', dest='force', action='store_true', help='force replay even for failing output asserts')
    parser.add_argument('--config', nargs='+', help='force alternative configuration file')
    parser.add_argument('--module', help='module name for analysis', required=True)  # TODO default "all"
    parser.add_argument('--verbose', '-v', help="verbose mode", action='store_true')
    parser.add_argument('--draw', help="draw debug results", action='store_true')
    parser.add_argument('--debug', help="print debug info about I/O streams", action='store_true')
    args = parser.parse_args()

    module_instance = replay(args)
    module_instance.verbose = args.verbose

    module_instance.start()
    # now wait until the module is alive
    module_instance.join()
    if not args.force:
        print("maximum delay:", module_instance.bus.max_delay)

    if args.draw:
        module_instance.draw()


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
