import unittest
from unittest.mock import MagicMock, call

from subt.artf_gas import ArtifactGasDetector
from subt.artifacts import GAS


class ArtifactGasDetectorTest(unittest.TestCase):

    def test_report(self):
        bus = bus=MagicMock()
        detector = ArtifactGasDetector(bus=bus, config={})
        detector.on_gas_detected(False)
        bus.publish.assert_not_called()

        detector.on_gas_detected(True)
        bus.publish.assert_called()
        bus.publish.assert_has_calls([call('artf', [GAS, [0, 0, 0]])])

# vim: expandtab sw=4 ts=4
