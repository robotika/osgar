"""
  Filter of detector outputs to unique reports
"""

from osgar.node import Node
from osgar.bus import BusShutdownException
from subt.trace import distance3D
from subt.artifacts import DRILL, GAS


class ArtifactFilter(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("artf_xyz")
        self.min_observations = config.get('min_observations', 2)
        self.robot_name = None  # for "signature" who discovered the artifact
        self.artifacts = []
        self.num_observations = []
        self.breadcrumbs = []
        self.verbose = False
        self.debug_artf = []
        self.debug_reported = []

    def publish_single_artf_xyz(self, artifact_data, pos):
        ax, ay, az = pos
        self.bus.publish('artf_xyz', [
            [artifact_data, [round(ax * 1000), round(ay * 1000), round(az * 1000)], self.robot_name, None]])
        if self.verbose:
            self.debug_reported.append((artifact_data, pos))

    def register_new_artifact(self, artifact_data, artifact_xyz):
        """
        Register newly detected artifact and update internal structures
        :param artifact_data: type of artifact
        :param artifact_xyz: artifact 3D position
        :return: True if artifact should be new published
        """
        # the breadcrumbs are sometimes wrongly classified as DRILL
        if artifact_data == DRILL:
            for x, y, z in self.breadcrumbs:
                if distance3D((x, y, z), artifact_xyz) < 4.0:
                    if self.verbose:
                        print('False detection - dist:', distance3D((x, y, z), artifact_xyz))
                    return False

        if self.verbose:
            self.debug_artf.append((artifact_data, artifact_xyz))

        for i, (stored_data, (x, y, z)) in enumerate(self.artifacts):
            if distance3D((x, y, z), artifact_xyz) < 4.0:
                # in case of uncertain type, rather report both
                if stored_data == artifact_data:
                    self.num_observations[i] += 1
                    # return true only when confirmation threshold was reached
                    if self.verbose:
                        print('Confirmed:', artifact_data, self.num_observations[i])
                    if artifact_data == GAS:
                        return False  # ignore confirmation - virtual detector is perfect
                    return self.num_observations[i] == self.min_observations  # report only once
        self.artifacts.append((artifact_data, artifact_xyz))
        self.num_observations.append(1)
        if self.min_observations > 1 and artifact_data != GAS:
            # new GAS should be reported independently on confirmation level
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
            if self.register_new_artifact(artifact_data, (ax, ay, az)):
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

    def draw(self):
        import matplotlib.pyplot as plt

        x = [xyz[0] for _, xyz in self.debug_artf]
        y = [xyz[1] for _, xyz in self.debug_artf]
        plt.plot(x, y, 'x')

        x = [xyz[0] for _, xyz in self.debug_reported]
        y = [xyz[1] for _, xyz in self.debug_reported]
        plt.scatter([x], [y], s=100, color='r')

        plt.axes().set_aspect('equal', 'datalim')
        plt.show()

# vim: expandtab sw=4 ts=4
