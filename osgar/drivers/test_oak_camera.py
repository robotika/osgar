import unittest
from unittest.mock import MagicMock, patch

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

    def test_config_driven_modifiers(self):
        from osgar.bus import Bus
        logger = MagicMock()
        logger.register = MagicMock(side_effect=range(1, 100))

        bus = Bus(logger)
        handler = bus.handle('test')
        # Configure output overrides:
        # depth: overrides default depth:gz to be uncompressed
        # nn_mask:null overrides default nn_mask:gz to be dropped
        # redroad is left default (so it should remain redroad:gz, i.e., compressed)
        handler.config_out = ["depth:", "nn_mask:null"]

        cam = OakCamera({}, bus=handler)

        self.assertIn("depth", handler.stream_id)
        self.assertIn("nn_mask", handler.stream_id)
        self.assertIn("redroad", handler.stream_id)

        depth_idx = handler.stream_id["depth"]
        nn_mask_idx = handler.stream_id["nn_mask"]
        redroad_idx = handler.stream_id["redroad"]

        self.assertNotIn(depth_idx, handler.compressed_output)
        self.assertNotIn(depth_idx, handler.no_output)
        self.assertIn(nn_mask_idx, handler.no_output)
        self.assertNotIn(nn_mask_idx, handler.compressed_output)
        self.assertIn(redroad_idx, handler.compressed_output)
        self.assertNotIn(redroad_idx, handler.no_output)

        # Do the same check for OakCameraV3
        logger_v3 = MagicMock()
        logger_v3.register = MagicMock(side_effect=range(1, 100))
        bus_v3 = Bus(logger_v3)
        handler_v3 = bus_v3.handle('test')
        handler_v3.config_out = ["depth:", "nn_mask:null"]

        cam3 = OakCameraV3({}, bus=handler_v3)

        self.assertIn("depth", handler_v3.stream_id)
        self.assertIn("nn_mask", handler_v3.stream_id)
        self.assertIn("redroad", handler_v3.stream_id)

        depth_idx_v3 = handler_v3.stream_id["depth"]
        nn_mask_idx_v3 = handler_v3.stream_id["nn_mask"]
        redroad_idx_v3 = handler_v3.stream_id["redroad"]

        self.assertNotIn(depth_idx_v3, handler_v3.compressed_output)
        self.assertNotIn(depth_idx_v3, handler_v3.no_output)
        self.assertIn(nn_mask_idx_v3, handler_v3.no_output)
        self.assertNotIn(nn_mask_idx_v3, handler_v3.compressed_output)
        self.assertIn(redroad_idx_v3, handler_v3.compressed_output)
        self.assertNotIn(redroad_idx_v3, handler_v3.no_output)

# vim: expandtab sw=4 ts=4
