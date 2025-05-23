import unittest
from unittest.mock import MagicMock, patch
from datetime import timedelta

# depthai package is not necessarily available and it is not needed for this unittest
with patch.dict('sys.modules', {'depthai': MagicMock()}):
    from osgar.drivers.oak_camera import OakCamera

class OakCameraTest(unittest.TestCase):

    def test_usage(self):
        config = {}
        handler = MagicMock()
        cam = OakCamera(config, bus=handler)

    def test_nn_image_size(self):
        config = {
            'nn_config': {
                'input_size': '160x120'
            }
        }
        handler = MagicMock()
        cam = OakCamera(config, bus=handler)
        self.assertEqual(cam.oak_config_nn_image_size, (160, 120))

# vim: expandtab sw=4 ts=4
