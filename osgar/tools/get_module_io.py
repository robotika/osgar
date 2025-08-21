"""
Get OSGAR Node module I/O names
"""
from unittest.mock import MagicMock

from osgar.lib.config import get_class_by_name


def get_module_io(module_name):
    klass = get_class_by_name(module_name)
    inputs = [o[3:] for o in dir(klass) if o.startswith('on_')]

    bus = MagicMock()
    inst = klass(bus=bus, config={})

    outputs = [c.args[0] for c in bus.mock_calls]
    return inputs, outputs


if __name__ == '__main__':
    import sys
    name = sys.argv[1]
    print(name, get_module_io(name))
