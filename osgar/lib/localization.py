"""
    TODO
"""

import osgar.lib.quaternion as quaternion


class Localization:
    def __init__(self):
        self.last_orientation = None
        self.last_position = None
        self.xyz = [0, 0, 0]

    def update_position(self, position):  # gps
        self.last_position = position

    def update_orientation(self, orientation):
        self.last_orientation = orientation

    def get_pose3d(self, dist):
        if self.last_orientation is None:
            return
        dist3d = quaternion.rotate_vector([dist, 0, 0], self.last_orientation)
        self.xyz = [a + b for a, b in zip(self.xyz, dist3d)]
        pose3d = [self.xyz, self.last_orientation]

        if self.last_position:
            pass  # add gps

        return pose3d
