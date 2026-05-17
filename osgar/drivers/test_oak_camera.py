import unittest
from unittest.mock import MagicMock, patch
from datetime import timedelta

# depthai package is not necessarily available and it is not needed for this unittest
dai_mock = MagicMock()
dai_mock.__version__ = '2.29.0.0'  # fake older version
with patch.dict('sys.modules', {'depthai': dai_mock}):
    from osgar.drivers.oak_camera import OakCamera
    from osgar.drivers.oak_camera_v3 import OakCamera as OakCameraV3

class OakCameraTest(unittest.TestCase):

    def test_usage(self):
        config = {}
        handler = MagicMock()
        cam = OakCamera(config, bus=handler)
        cam3 = OakCameraV3(config, bus=handler)

    def test_nn_image_size(self):
        config = {
            'model': 'dummy_path',
            'nn_config': {
                'input_size': '160x120'
            }
        }
        handler = MagicMock()
        cam = OakCamera(config, bus=handler)
        if hasattr(cam, 'models'):
            self.assertEqual(cam.models[0]['nn_config']['input_size'], '160x120')
        else:
            self.assertEqual(cam.oak_config_nn_image_size, (160, 120))
            
        cam3 = OakCameraV3(config, bus=handler)
        self.assertEqual(cam3.models[0]['nn_config']['input_size'], '160x120')

    def test_imu_records(self):
        config = {
            'is_imu_enabled': True,
            'number_imu_records': 15,
            'disable_magnetometer_fusion': True
        }
        handler = MagicMock()
        cam = OakCamera(config, bus=handler)
        self.assertEqual(cam.number_imu_records, 15)
        self.assertEqual(cam.disable_magnetometer_fusion, True)

        cam3 = OakCameraV3(config, bus=handler)
        self.assertEqual(cam3.number_imu_records, 15)
        self.assertEqual(cam3.disable_magnetometer_fusion, True)

# vim: expandtab sw=4 ts=4
