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


# Exposure is configured per source: Source1 and Source2 are the stereo pair (depth), Source3 is the color sensor.
# The two stereo sources are linked, setting Source1 applies to both.
EXPOSURE_SOURCES = {"depth": "Source1", "color": "Source3"}


class STMCamera:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus
        # per stream (depth_, color_) exposure settings, None = auto exposure
        #   <stream>_exposure          manual exposure time in us (disables auto exposure)
        #                              depth: 1-165000, color: 1-1665
        #   <stream>_gain              manual gain (disables auto exposure)
        #                              depth: 16-248, color: 16-128
        #   <stream>_ae_max_exposure   upper limit of exposure time in us for auto exposure
        #                              depth: 1-166000, color: 1-1665
        #   <stream>_ae_priority       auto exposure priority, 0 or 1 (default 0), only effective in auto exposure,
        #                              1 gives a darker image than 0 in a bright scene
        #   depth_ae_mean_intensity    target mean image intensity for auto exposure, 0-255, depth only
        self.exposure = {}
        for stream in EXPOSURE_SOURCES:
            self.exposure[stream] = {key: config.get(f"{stream}_{key}")
                                     for key in ("exposure", "gain", "ae_max_exposure", "ae_mean_intensity")}
            self.exposure[stream]["ae_priority"] = config.get(f"{stream}_ae_priority", 0)
        if self.exposure["color"]["ae_mean_intensity"] is not None:
            raise ValueError("color_ae_mean_intensity is not supported by the camera")
        # color image settings
        #   color_gamma                gamma, 100-500 (default 300), higher value gives a brighter image
        #   color_white_balance        "auto" (default) or color temperature in K, 2800-6500
        self.gamma = config.get("color_gamma", 300)
        self.white_balance = config.get("color_white_balance", "auto")
        self.fps = config.get("fps")  # 6-60, None = camera default (30)
        self.timeout_ms = config.get("timeout_ms", 1000)
        self.bus.register('color', 'depth')
        self.camera = None

    def set_exposure(self, stream):
        settings = self.exposure[stream]
        self.camera.SourceSelector.SetValue(EXPOSURE_SOURCES[stream])
        # the mode is always set explicitly, the camera keeps it between runs
        self.camera.ExposureAuto.SetValue("Continuous")
        self.camera.BslAutoExposurePriority.SetValue(int(settings["ae_priority"]))
        if settings["ae_max_exposure"] is not None:
            self.camera.BslAEMaxExposure.SetValue(float(settings["ae_max_exposure"]))
        if settings["ae_mean_intensity"] is not None:
            self.camera.BslMeanIntensity.SetValue(float(settings["ae_mean_intensity"]))
        if settings["exposure"] is not None or settings["gain"] is not None:
            self.camera.ExposureAuto.SetValue("Off")
            if settings["exposure"] is not None:
                self.camera.ExposureTime.SetValue(float(settings["exposure"]))
            if settings["gain"] is not None:
                self.camera.Gain.SetValue(float(settings["gain"]))
        g_logger.info("%s: %s exposure, exposure time %.0f us, gain %.1f", stream,
                      self.camera.ExposureAuto.GetValue(), self.camera.ExposureTime.GetValue(),
                      self.camera.Gain.GetValue())

    def set_color_image(self):
        self.camera.SourceSelector.SetValue(EXPOSURE_SOURCES["color"])
        self.camera.Gamma.SetValue(float(self.gamma))
        if self.white_balance == "auto":
            self.camera.BalanceWhiteAuto.SetValue("Continuous")
        else:
            self.camera.BalanceWhiteAuto.SetValue("Off")
            self.camera.BalanceRatio.SetValue(float(self.white_balance))
        g_logger.info("color: gamma %.0f, white balance %s (%.0f K)", self.camera.Gamma.GetValue(),
                      self.camera.BalanceWhiteAuto.GetValue(), self.camera.BalanceRatio.GetValue())

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
            for stream in EXPOSURE_SOURCES:
                self.set_exposure(stream)
            self.set_color_image()
            if self.fps is not None:
                self.camera.AcquisitionFrameRate.SetValue(float(self.fps))
            g_logger.info("Frame rate: %.1f fps", self.camera.AcquisitionFrameRate.GetValue())
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
