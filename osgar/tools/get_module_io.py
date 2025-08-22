"""
Get OSGAR Node module I/O names
"""
from unittest.mock import MagicMock

from osgar.lib.config import get_class_by_name


def get_module_io(module_name):
    klass = get_class_by_name(module_name)
    inputs = [o[3:] for o in dir(klass) if o.startswith('on_')]

    bus = MagicMock()
    config = MagicMock()
    try:
        klass(bus=bus, config={})
    except KeyError:
        pass  # due to missing required config parameters

    outputs = [c.args[0] for c in bus.mock_calls]
    return inputs, outputs


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', help='name of the Node class like "doctor:Doctor"', required=True)
    args = parser.parse_args()

    print('Module:', args.name)
    i,o = get_module_io(args.name)
    print('Inputs:', i)
    print('Outputs:', o)
