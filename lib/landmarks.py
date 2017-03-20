"""
  Laser landmarks detector
"""


class ConeLandmarks(object):

    def __init__(self, raw_laser_data, verbose=False):
        self.data = raw_laser_data
        self.verbose = verbose

    def find_cones(self):
        return []

# vim: expandtab sw=4 ts=4
