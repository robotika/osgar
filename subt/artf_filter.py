"""
  Filter of detector outputs to unique reports
"""

from osgar.node import Node
from osgar.bus import BusShutdownException
from subt.trace import distance3D
from subt.artifacts import DRILL


class ArtifactFilter(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("artf_xyz")
        self.num_confirm = config.get('num_confirm', 0)
        self.robot_name = None  # for "signature" who discovered the artifact
        self.artifacts = []
        self.confirmations = []
        self.breadcrumbs = []
        self.verbose = False

    def publish_single_artf_xyz(self, artifact_data, pos):
        ax, ay, az = pos
        self.bus.publish('artf_xyz', [
            [artifact_data, [round(ax * 1000), round(ay * 1000), round(az * 1000)], self.robot_name, None]])

    def maybe_remember_artifact(self, artifact_data, artifact_xyz):
        # the breadcrumbs are sometimes wrongly classified as DRILL
        if artifact_data == DRILL:
            for x, y, z in self.breadcrumbs:
                if distance3D((x, y, z), artifact_xyz) < 4.0:
                    if self.verbose:
                        print('False detection - dist:', distance3D((x, y, z), artifact_xyz))
                    return False

        for i, (stored_data, (x, y, z)) in enumerate(self.artifacts):
            if distance3D((x, y, z), artifact_xyz) < 4.0:
                # in case of uncertain type, rather report both
                if stored_data == artifact_data:
                    self.confirmations[i] += 1
                    # return true only when confirmation threshold was reached
                    if self.verbose:
                        print('Confirmed:', artifact_data, self.confirmations[i])
                    return self.confirmations[i] == self.num_confirm
        self.artifacts.append((artifact_data, artifact_xyz))
        self.confirmations.append(0)
        if self.num_confirm > 0:
            return False
        return True

    def handle_artf(self, artifact_data, world_xyz):
        ax, ay, az = world_xyz
        if -20 < ax < 0 and -10 < ay < 10:  # AND of currently available staging areas
            # Urban (-20 < ax < 0 and -10 < ay < 10)
            # Cave  (-50 < ax < 0 and -25 < ay < 25)
            # filter out elements on staging area
            if self.verbose:
                print(self.time, 'Robot at staging area:', (ax, ay, az))
        else:
            if self.maybe_remember_artifact(artifact_data, (ax, ay, az)):
                self.publish_single_artf_xyz(artifact_data, (ax, ay, az))

    def on_robot_name(self, data):
        self.robot_name = data.decode('ascii')

    def on_localized_artf(self, data):
        if self.verbose:
            print(self.time, data)
        self.handle_artf(*data)

    def on_breadcrumb(self, data):
        self.breadcrumbs.append(data)
        if self.verbose:
            print('Breadcrumb:', data)

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # unknown channel

# vim: expandtab sw=4 ts=4
