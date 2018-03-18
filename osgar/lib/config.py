"""
  Osgar Config Class
"""
import json


ROBOT_CONTAINER_VER = 2
SUPPORTED_VERSIONS = [ROBOT_CONTAINER_VER]


class MergeConflictError(Exception):
    pass


def load(*filenames):
    ret = {}
    for filename in filenames:
        with open(filename) as f:
            data = json.load(f)
            assert 'version' in data, data
            assert data['version'] in SUPPORTED_VERSIONS, data['version']
            ret = merge_dict(ret, data)
    return ret


def merge_dict(dict1, dict2):
    if not isinstance(dict1, dict) or not isinstance(dict2, dict):
        raise MergeConflictError(str((dict1, dict2)))
    ret = dict1.copy()
    for key in dict2.keys():
        if key in dict1:
            if dict1[key] != dict2[key]:
                ret[key] = merge_dict(dict1[key], dict2[key])
        else:
            ret[key] = dict2[key]
    return ret

# vim: expandtab sw=4 ts=4
