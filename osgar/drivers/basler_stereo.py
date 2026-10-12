"""
    Driver for Basler STM cameras, https://www.baslerweb.com/en/cameras/basler-stereo-camera/stereo-mini/
    Implemented via Basler sdk and pypylon. The first detected camera is used with default settings.
    Logged outputs:
      - color: JPEG bytes (from the Intensity component)
      - depth: uint16 numpy array, distance in millimeters (Z of the Range component, 0 = invalid)
"""

from threading import Thread
import logging
import numpy as np
import cv2

g_logger = logging.getLogger(__name__)

try:
    from pypylon import pylon
except Exception:
    pylon = None
    g_logger.error("pypylon is not installed!")


def component_by_type(container, component_type):
    for i in range(container.DataComponentCount):
        component = container.GetDataComponentByIndex(i)
        if component.ComponentType == component_type:
            return component
    return None


def intensity_to_bgr(component):
    # Intensity is RGBA8 packed, which is not supported by the component Array accessor
    rgba = np.frombuffer(component.GetMemoryView(), dtype=np.uint8)
    rgba = rgba.reshape(component.Height, component.Width, 4)
    return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)


def range_to_depth(component):
    # Range is Coord3D_ABC32f, Z (third channel) is the distance in millimeters
    z = component.Array[:, :, 2]
    valid = np.isfinite(z) & (z > 0)
    return np.where(valid, np.clip(z, 0, 65535), 0).astype(np.uint16)


class STMCamera:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus
        self.timeout_ms = config.get("timeout_ms", 1000)
        self.bus.register('color', 'depth')
        self.camera = None

    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        if pylon is None:
            self.bus.shutdown()
            return

        factory = pylon.TlFactory.GetInstance()
        devices = factory.EnumerateDevices()
        if len(devices) == 0:
            g_logger.error("No Basler camera detected.")
            self.bus.shutdown()
            return

        self.camera = pylon.InstantCamera(factory.CreateDevice(devices[0]))
        try:
            self.camera.Open()
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            while self.bus.is_alive() and self.camera.IsGrabbing():
                grab = self.camera.RetrieveResult(self.timeout_ms, pylon.TimeoutHandling_Return)
                try:
                    if not grab.IsValid() or not grab.GrabSucceeded():
                        continue
                    container = grab.GetDataContainer()
                    intensity = component_by_type(container, pylon.ComponentType_Intensity)
                    range_data = component_by_type(container, pylon.ComponentType_Range)
                    if intensity is not None:
                        ok, data = cv2.imencode('.jpg', intensity_to_bgr(intensity))
                        if ok:
                            self.bus.publish("color", data.tobytes())
                    if range_data is not None:
                        self.bus.publish("depth", range_to_depth(range_data))
                finally:
                    grab.Release()
        finally:
            if self.camera.IsGrabbing():
                self.camera.StopGrabbing()
            if self.camera.IsOpen():
                self.camera.Close()
            self.camera = None

    def request_stop(self):
        self.bus.shutdown()
