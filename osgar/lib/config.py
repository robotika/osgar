"""
  Osgar Config Class
"""
import json
import numbers
import sys
from importlib import import_module
from ast import literal_eval

from osgar.drivers import all_drivers


ROBOT_CONTAINER_VER = 2
SUPPORTED_VERSIONS = [ROBOT_CONTAINER_VER]


def get_class_by_name(name):
    if name in all_drivers:
        name = all_drivers[name]
    assert ':' in name, name  # import path and class name expected
    s = name.split(':')
    assert len(s) == 2  # package and class name
    module_name, class_name = s
    m = import_module(module_name)
    return getattr(m, class_name)


class MergeConflictError(Exception):
    pass


def _application2import(application):
    # ModuleSpec: https://www.python.org/dev/peps/pep-0451/
    # __qualname__: https://www.python.org/dev/peps/pep-3155/
    s = sys.modules[application.__module__].__spec__
    return f"{s.name}:{application.__qualname__}"


def config_load(*filenames, application=None, params=None, without=None):
    ret = {}
    for filename in filenames:
        with open(filename) as f:
            data = json.load(f)
            assert 'version' in data, data
            assert data['version'] in SUPPORTED_VERSIONS, data['version']
            ret = merge_dict(ret, data)
    if params is not None:
        for param in params:
            assert '=' in param, param
            key_path, str_value = param.split('=')
            key = key_path.split('.')
            assert len(key) == 2, key
            ret['robot']['modules'][key[0]]['init'][key[1]] = literal_eval(str_value)
    if without is not None:
        for module in without:
            assert module in ret['robot']['modules'], f"Module {module} not in {ret['robot']['modules'].keys()}"
            del ret['robot']['modules'][module]
            new_links = []
            key = module + '.'
            for link_from, link_to in ret['robot']['links']:
                if link_from.startswith(key) or link_to.startswith(key):
                    continue
                new_links.append([link_from, link_to])
            ret['robot']['links'] = new_links
    if application is None:
        return ret
    if not isinstance(application, str):
        application = _application2import(application)
    for module_name, module_config in ret['robot']['modules'].items():
        if module_config['driver'] == 'application':
            module_config['driver'] = application
    return ret


def merge_dict(dict1, dict2):
    ret = dict1.copy()
    for key, val2 in dict2.items():
        try:
            val1 = dict1[key]
            if isinstance(val1, dict) and isinstance(val2, dict):
                ret[key] = merge_dict(val1, val2)
            else:
                for t in numbers.Number, str, list, tuple:
                    if isinstance(val1, t) and isinstance(val2, t):
                        ret[key] = val2
                        break
                else:
                    raise MergeConflictError(str((val1, val2)))
        except KeyError:
            ret[key] = val2
    return ret

# vim: expandtab sw=4 ts=4
