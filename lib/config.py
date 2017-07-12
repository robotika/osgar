"""
  Osgar Config Class
"""
import json


class Config(object):

    SUPPORTED_VERSION = 1

    @classmethod
    def load(cls, filename):
        cls.data = json.load(open(filename))
        assert 'version' in cls.data, cls.data
        assert cls.data['version'] == cls.SUPPORTED_VERSION, cls.data['version']
        cls.version = cls.data['version']

        return cls

    def __init__(self):
        self.data = {}

# vim: expandtab sw=4 ts=4
