import unittest
from unittest.mock import MagicMock, call

from subt.artf_gas import ArtifactGasDetector
from subt.artifacts import GAS


class ArtifactGasDetectorTest(unittest.TestCase):

    def test_report(self):
        bus = bus=MagicMock()
        detector = ArtifactGasDetector(bus=bus, config={})
        detector.on_pose3d([[1, 2, 3], [0, 0, 0, 1]])
        detector.on_gas_detected(False)
        bus.publish.assert_not_called()

        detector.on_pose3d([[1, 0, 1], [0, 0, 0, 1]])
        detector.on_gas_detected(True)
        bus.publish.assert_called()
        bus.publish.assert_has_calls([call('localized_artf', [GAS, [1.0, 0, 1.0]])])

    def test_drone_z(self):
        # the location point is on the ground level, which is OK for ground vehicles but not for drones
        bus = bus=MagicMock()
        detector = ArtifactGasDetector(bus=bus, config={})
        detector.on_pose3d([[1, 2, 3], [0, 0, 0, 1]])
        detector.on_gas_detected(False)
        detector.on_pose3d([[1, 0, 1], [0, 0, 0, 1]])
        detector.on_bottom_scan([1.0])  # top/bottom scans are in meters

        bus.publish.reset_mock()
        detector.on_gas_detected(True)
        bus.publish.assert_called()
        bus.publish.assert_has_calls([call('localized_artf', [GAS, [1.0, 0, 1.0 - 1.0]])])


# vim: expandtab sw=4 ts=4
