import unittest
from unittest.mock import MagicMock, patch

import numpy as np
import cv2

from osgar.drivers.basler_stereo import (STMCamera, component_by_type, intensity_to_bgr, range_to_depth,
                                         EXPOSURE_SOURCES)

INTENSITY = 1
RANGE = 4


def make_component(component_type, width=4, height=2, array=None, rgba=None):
    c = MagicMock()
    c.ComponentType = component_type
    c.Width = width
    c.Height = height
    if array is not None:
        c.Array = array
    if rgba is not None:
        c.GetMemoryView.return_value = memoryview(rgba.tobytes())
    return c


def make_container(*components):
    container = MagicMock()
    container.DataComponentCount = len(components)
    container.GetDataComponentByIndex.side_effect = lambda i: components[i]
    return container


class ConversionTest(unittest.TestCase):
    def test_component_by_type(self):
        a, b = make_component(INTENSITY), make_component(RANGE)
        container = make_container(a, b)
        self.assertIs(component_by_type(container, RANGE), b)
        self.assertIs(component_by_type(container, INTENSITY), a)
        self.assertIsNone(component_by_type(container, 6))

    def test_intensity_to_bgr(self):
        rgba = np.zeros((2, 3, 4), dtype=np.uint8)
        rgba[:, :, 0] = 10  # R
        rgba[:, :, 1] = 20  # G
        rgba[:, :, 2] = 30  # B
        rgba[:, :, 3] = 255
        bgr = intensity_to_bgr(make_component(INTENSITY, width=3, height=2, rgba=rgba))
        self.assertEqual(bgr.shape, (2, 3, 3))
        self.assertEqual(list(bgr[0, 0]), [30, 20, 10])

    def test_range_to_depth(self):
        xyz = np.zeros((1, 6, 3), dtype=np.float32)
        xyz[0, :, 2] = [1000.4, 0, -5, np.nan, np.inf, 70000]
        depth = range_to_depth(make_component(RANGE, width=6, height=1, array=xyz))
        self.assertEqual(depth.dtype, np.uint16)
        self.assertEqual(depth.shape, (1, 6))
        # invalid (zero, negative, nan, inf) values are 0, too far values are clipped
        self.assertEqual(list(depth[0]), [1000, 0, 0, 0, 0, 65535])


class ConfigTest(unittest.TestCase):
    def test_defaults(self):
        bus = MagicMock()
        cam = STMCamera({}, bus)
        bus.register.assert_called_with('color', 'depth')
        self.assertIsNone(cam.fps)
        self.assertEqual(cam.gamma, 300)
        self.assertEqual(cam.white_balance, "auto")
        for stream in EXPOSURE_SOURCES:
            self.assertEqual(cam.exposure[stream]["ae_priority"], 0)
            self.assertIsNone(cam.exposure[stream]["exposure"])

    def test_color_mean_intensity_not_supported(self):
        with self.assertRaises(ValueError):
            STMCamera({"color_ae_mean_intensity": 100}, MagicMock())


class SettingsTest(unittest.TestCase):
    def setUp(self):
        self.camera = MagicMock()
        self.camera.ExposureAuto.GetValue.return_value = "Continuous"
        self.camera.ExposureTime.GetValue.return_value = 100.0
        self.camera.Gain.GetValue.return_value = 16.0
        self.camera.Gamma.GetValue.return_value = 300.0
        self.camera.BalanceWhiteAuto.GetValue.return_value = "Continuous"
        self.camera.BalanceRatio.GetValue.return_value = 4600.0

    def create(self, config):
        cam = STMCamera(config, MagicMock())
        cam.camera = self.camera
        return cam

    def test_auto_exposure_default(self):
        cam = self.create({})
        for stream, source in EXPOSURE_SOURCES.items():
            self.camera.reset_mock()
            cam.set_exposure(stream)
            self.camera.SourceSelector.SetValue.assert_called_once_with(source)
            self.camera.ExposureAuto.SetValue.assert_called_once_with("Continuous")
            self.camera.BslAutoExposurePriority.SetValue.assert_called_once_with(0)
            self.assertFalse(self.camera.ExposureTime.SetValue.called)
            self.assertFalse(self.camera.Gain.SetValue.called)
            self.assertFalse(self.camera.BslAEMaxExposure.SetValue.called)
            self.assertFalse(self.camera.BslMeanIntensity.SetValue.called)

    def test_auto_exposure_parameters(self):
        cam = self.create({"depth_ae_max_exposure": 20000, "depth_ae_mean_intensity": 90, "depth_ae_priority": 1})
        cam.set_exposure("depth")
        self.camera.SourceSelector.SetValue.assert_called_once_with("Source1")
        self.camera.BslAEMaxExposure.SetValue.assert_called_once_with(20000.0)
        self.camera.BslMeanIntensity.SetValue.assert_called_once_with(90.0)
        self.camera.BslAutoExposurePriority.SetValue.assert_called_once_with(1)
        self.assertEqual(self.camera.ExposureAuto.SetValue.call_args_list[-1].args, ("Continuous",))

    def test_manual_exposure(self):
        cam = self.create({"color_exposure": 800, "color_gain": 32})
        cam.set_exposure("color")
        self.camera.SourceSelector.SetValue.assert_called_once_with("Source3")
        self.assertEqual(self.camera.ExposureAuto.SetValue.call_args_list[-1].args, ("Off",))
        self.camera.ExposureTime.SetValue.assert_called_once_with(800.0)
        self.camera.Gain.SetValue.assert_called_once_with(32.0)

    def test_manual_gain_only(self):
        cam = self.create({"depth_gain": 64})
        cam.set_exposure("depth")
        self.assertEqual(self.camera.ExposureAuto.SetValue.call_args_list[-1].args, ("Off",))
        self.assertFalse(self.camera.ExposureTime.SetValue.called)
        self.camera.Gain.SetValue.assert_called_once_with(64.0)

    def test_color_image_default(self):
        cam = self.create({})
        cam.set_color_image()
        self.camera.SourceSelector.SetValue.assert_called_once_with("Source3")
        self.camera.Gamma.SetValue.assert_called_once_with(300.0)
        self.camera.BalanceWhiteAuto.SetValue.assert_called_once_with("Continuous")
        self.assertFalse(self.camera.BalanceRatio.SetValue.called)

    def test_color_image_manual(self):
        cam = self.create({"color_gamma": 500, "color_white_balance": 2800})
        cam.set_color_image()
        self.camera.Gamma.SetValue.assert_called_once_with(500.0)
        self.camera.BalanceWhiteAuto.SetValue.assert_called_once_with("Off")
        self.camera.BalanceRatio.SetValue.assert_called_once_with(2800.0)


class RunInputTest(unittest.TestCase):
    def make_grab(self, valid=True, succeeded=True):
        rgba = np.full((2, 4, 4), 128, dtype=np.uint8)
        xyz = np.zeros((2, 4, 3), dtype=np.float32)
        xyz[:, :, 2] = 1500
        container = make_container(make_component(RANGE, array=xyz),
                                   make_component(INTENSITY, rgba=rgba))
        grab = MagicMock()
        grab.IsValid.return_value = valid
        grab.GrabSucceeded.return_value = succeeded
        grab.GetDataContainer.return_value = container
        return grab

    def make_bus(self, frames):
        bus = MagicMock()
        bus.is_alive.side_effect = [True] * frames + [False]
        return bus

    def make_pylon(self, grabs, devices=1):
        pylon = MagicMock()
        pylon.ComponentType_Intensity = INTENSITY
        pylon.ComponentType_Range = RANGE
        factory = pylon.TlFactory.GetInstance.return_value
        factory.EnumerateDevices.return_value = [MagicMock() for __ in range(devices)]
        camera = pylon.InstantCamera.return_value
        camera.IsGrabbing.return_value = True
        camera.RetrieveResult.side_effect = grabs
        return pylon, camera

    def test_without_pypylon(self):
        bus = MagicMock()
        with patch('osgar.drivers.basler_stereo.pylon', None):
            STMCamera({}, bus).run_input()
        bus.shutdown.assert_called_once()
        self.assertFalse(bus.publish.called)

    def test_no_camera(self):
        bus = MagicMock()
        pylon, camera = self.make_pylon([], devices=0)
        with patch('osgar.drivers.basler_stereo.pylon', pylon):
            STMCamera({}, bus).run_input()
        bus.shutdown.assert_called_once()
        self.assertFalse(pylon.InstantCamera.called)

    def test_publish_color_and_depth(self):
        grab = self.make_grab()
        bus = self.make_bus(1)
        pylon, camera = self.make_pylon([grab])
        with patch('osgar.drivers.basler_stereo.pylon', pylon):
            STMCamera({}, bus).run_input()

        channels = [call.args[0] for call in bus.publish.call_args_list]
        self.assertEqual(channels, ['color', 'depth'])
        color = cv2.imdecode(np.frombuffer(bus.publish.call_args_list[0].args[1], dtype=np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(color.shape, (2, 4, 3))
        depth = bus.publish.call_args_list[1].args[1]
        self.assertEqual(depth.dtype, np.uint16)
        self.assertTrue((depth == 1500).all())
        grab.Release.assert_called_once()

    def test_first_device_and_fps(self):
        bus = self.make_bus(0)
        pylon, camera = self.make_pylon([], devices=2)
        factory = pylon.TlFactory.GetInstance.return_value
        devices = factory.EnumerateDevices.return_value
        camera.AcquisitionFrameRate.GetValue.return_value = 10.0
        with patch('osgar.drivers.basler_stereo.pylon', pylon):
            STMCamera({"fps": 10}, bus).run_input()
        factory.CreateDevice.assert_called_once_with(devices[0])
        camera.AcquisitionFrameRate.SetValue.assert_called_once_with(10.0)
        camera.StartGrabbing.assert_called_once()

    def test_invalid_grab_skipped(self):
        grabs = [self.make_grab(valid=False), self.make_grab(succeeded=False)]
        bus = self.make_bus(2)
        pylon, camera = self.make_pylon(grabs)
        with patch('osgar.drivers.basler_stereo.pylon', pylon):
            STMCamera({}, bus).run_input()
        self.assertFalse(bus.publish.called)
        for grab in grabs:
            grab.Release.assert_called_once()

    def test_camera_closed_on_error(self):
        bus = self.make_bus(1)
        pylon, camera = self.make_pylon([])
        camera.IsOpen.return_value = True
        with patch('osgar.drivers.basler_stereo.pylon', pylon):
            cam = STMCamera({"color_gamma": 500}, bus)
            camera.Gamma.SetValue.side_effect = RuntimeError("out of range")
            with self.assertRaises(RuntimeError):
                cam.run_input()
        camera.Close.assert_called_once()
        self.assertIsNone(cam.camera)

# vim: expandtab sw=4 ts=4
