import unittest
from unittest.mock import MagicMock, patch, call
from datetime import timedelta

# depthai package is not necessarily available and it is not needed for this unittest
dai_mock = MagicMock()
dai_mock.__version__ = '2.29.0.0'  # fake older version
with patch.dict('sys.modules', {'depthai': dai_mock}):
    from osgar.drivers.oak_camera import OakCamera
    from osgar.drivers.oak_camera_v3 import OakCamera as OakCameraV3, dai

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

class OakCameraV3IMUTest(unittest.TestCase):

    def test_imu_outputs(self):
        for raw, orientation in [(False, True), (True, False), (True, True)]:
            for disable_fusion in [False, True]:
                with self.subTest(raw=raw, orientation=orientation, disable_fusion=disable_fusion):
                    bus = MagicMock()
                    cam = OakCameraV3({
                        'is_imu_enabled': True,
                        'imu_raw_data': raw,
                        'imu_orientation': orientation,
                        'disable_magnetometer_fusion': disable_fusion,
                    }, bus)
                    imu = MagicMock()
                    cam.configure_imu(imu)
                    sensors = []
                    if raw:
                        sensors.extend([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW])
                    if orientation:
                        sensors.append(dai.IMUSensor.GAME_ROTATION_VECTOR if disable_fusion
                                       else dai.IMUSensor.ROTATION_VECTOR)
                    imu.enableIMUSensor.assert_called_once_with(sensors, 100)

                    data = MagicMock()
                    for name in ['acceleroMeter', 'gyroscope', 'rotationVector']:
                        getattr(data, name).getTimestampDevice.return_value = timedelta(seconds=1)
                    data.acceleroMeter.x, data.acceleroMeter.y, data.acceleroMeter.z = 1, 2, 3
                    data.gyroscope.x, data.gyroscope.y, data.gyroscope.z = 4, 5, 6
                    data.rotationVector.rotationVectorAccuracy = 0.1
                    data.rotationVector.i, data.rotationVector.j = 0, 0
                    data.rotationVector.k, data.rotationVector.real = 0, 1
                    if not raw:
                        del data.acceleroMeter
                        del data.gyroscope
                    if not orientation:
                        del data.rotationVector
                    packet = MagicMock()
                    packet.packets = [data]
                    cam.publish_imu(packet)
                    expected = []
                    if raw:
                        expected.extend([
                            call('acc_raw_list', [[1.0, 1, 2, 3]]),
                            call('gyro_raw_list', [[1.0, 4, 5, 6]]),
                        ])
                    if orientation:
                        expected.append(call('orientation_list', [[1.0, 0.1, 0, 0, 0, 1]]))
                    self.assertEqual(bus.publish.call_args_list, expected)

    def test_defaults(self):
        cam = OakCameraV3({}, MagicMock())
        self.assertTrue(cam.imu_orientation)
        self.assertFalse(cam.imu_raw_data)

    def test_no_imu_outputs(self):
        with self.assertRaisesRegex(ValueError, 'IMU requires'):
            OakCameraV3({'is_imu_enabled': True, 'imu_orientation': False}, MagicMock())

# vim: expandtab sw=4 ts=4
