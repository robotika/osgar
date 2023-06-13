"""
    Osgar driver for Luxonis OAK cameras.
    https://www.luxonis.com/
"""

from threading import Thread
import depthai as dai
import logging

g_logger = logging.getLogger(__name__)


def cam_is_available(cam_ip):
    devices = dai.DeviceBootloader.getAllAvailableDevices()
    available_devices = []
    for dev in devices:
        if cam_ip == dev.desc.name:
            return True
        available_devices.append(dev.desc.name)

    g_logger.warning(f"IP {cam_ip} was not found!")
    g_logger.info(f'Found devices: {", ".join(available_devices)}')

    return False


class OakCamera:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus

        self.bus.register('depth', 'color', 'rotation_list')
        self.fps = config.get('fps', 10)
        print(self.fps)
        self.is_depth = config.get('is_depth', False)
        self.is_color = config.get('is_color', False)
        self.is_imu_rotation = config.get('is_imu_rotation', False)
        self.cam_ip = config.get('cam_ip')
        # Set camera IP via: https://github.com/luxonis/depthai-python/blob/main/examples/bootloader/poe_set_ip.py

        self.is_extended_disparity = config.get("stereo_extended_disparity", False)
        self.is_subpixel = config.get("stereo_subpixel", False)
        self.is_left_right_check = config.get("stereo_left_right_check", False)
        assert not(self.is_extended_disparity and self.is_subpixel)  # Do not use extended_disparity and subpixel together.

        self.color_manual_focus = config.get("color_manual_focus")

        mono_resolution_value = config.get("mono_resolution", "THE_400_P")
        assert mono_resolution_value in ["THE_400_P", "THE_480_P", "THE_720_P", "THE_800_P"], mono_resolution_value
        self.mono_resolution = getattr(dai.MonoCameraProperties.SensorResolution, mono_resolution_value)

        color_resolution_value = config.get("color_resolution", "THE_1080_P")
        assert color_resolution_value in["THE_1080_P", "THE_4_K", "THE_12_MP", "THE_13_MP"], color_resolution_value
        self.color_resolution = getattr(dai.ColorCameraProperties.SensorResolution, color_resolution_value)

        median_filter_value = config.get("stereo_median_filter", "KERNEL_7x7")
        assert median_filter_value in ["KERNEL_7x7", "KERNEL_5x5", "KERNEL_3x3", "MEDIAN_OFF"], median_filter_value
        self.median_filter = getattr(dai.MedianFilter, median_filter_value)

        stereo_mode_value = config.get("stereo_mode", "HIGH_DENSITY")
        assert stereo_mode_value in ["HIGH_DENSITY", "HIGH_ACCURACY"], stereo_mode_value
        self.stereo_mode = getattr(dai.node.StereoDepth.PresetMode, stereo_mode_value)



    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        pipeline = dai.Pipeline()
        queue_names = []

        # Define sources and outputs, set properties and linking nodes.
        if self.is_color:
            color = pipeline.create(dai.node.ColorCamera)
            color_encoder = pipeline.create(dai.node.VideoEncoder)
            color_out = pipeline.create(dai.node.XLinkOut)

            queue_names.append("color")
            color_out.setStreamName('color')

            color.setResolution(self.color_resolution)
            color.setBoardSocket(dai.CameraBoardSocket.RGB)
            color.setFps(self.fps)
            # Value 130 for manual focus allows align color and depth frames.
            # https://docs.luxonis.com/projects/api/en/latest/samples/StereoDepth/rgb_depth_aligned/
            if self.color_manual_focus is not None:
                color.initialControl.setManualFocus(self.color_manual_focus)

            color_encoder.setDefaultProfilePreset(self.fps, dai.VideoEncoderProperties.Profile.MJPEG)

            color.video.link(color_encoder.input)
            color_encoder.bitstream.link(color_out.input)

        if self.is_depth:
            left = pipeline.create(dai.node.MonoCamera)
            right = pipeline.create(dai.node.MonoCamera)
            stereo = pipeline.create(dai.node.StereoDepth)
            depth_out = pipeline.create(dai.node.XLinkOut)

            queue_names.append("depth")
            depth_out.setStreamName("depth")

            left.setResolution(self.mono_resolution)
            left.setBoardSocket(dai.CameraBoardSocket.LEFT)
            left.setFps(self.fps)
            right.setResolution(self.mono_resolution)
            right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
            right.setFps(self.fps)

            stereo.setDefaultProfilePreset(self.stereo_mode)
            # https://docs.luxonis.com/projects/api/en/latest/components/nodes/stereo_depth/#currently-configurable-blocks
            stereo.initialConfig.setMedianFilter(self.median_filter)
            stereo.setExtendedDisparity(self.is_extended_disparity)
            stereo.setSubpixel(self.is_subpixel)
            stereo.setLeftRightCheck(self.is_left_right_check)  # https://docs.luxonis.com/en/latest/pages/faq/#left-right-check-depth-mode

            left.out.link(stereo.left)
            right.out.link(stereo.right)
            stereo.depth.link(depth_out.input)

        if self.is_imu_rotation:
            imu = pipeline.create(dai.node.IMU)
            imu_out = pipeline.create(dai.node.XLinkOut)

            imu_out.setStreamName("rotation")
            queue_names.append("rotation")

            imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 100)
            imu.setBatchReportThreshold(20)
            imu.setMaxBatchReports(20)
            imu.out.link(imu_out.input)

        if not queue_names:
            g_logger.error("No stream enabled!")
            return


        if self.cam_ip and cam_is_available(self.cam_ip):  # USB cameras?
            device_info = dai.DeviceInfo()
            device_info.state = dai.XLinkDeviceState.X_LINK_BOOTLOADER
            device_info.desc.protocol = dai.XLinkProtocol.X_LINK_TCP_IP
            device_info.desc.name = self.cam_ip
        else:
            device_info = None
            g_logger.info("Used the first available device.")

        # Connect to device and start pipeline
        with dai.Device(pipeline, device_info) as device:
            while self.bus.is_alive():
                queue_events = device.getQueueEvents(queue_names)

                for queue_name in queue_events:
                    packets = device.getOutputQueue(queue_name).tryGetAll()
                    if len(packets) > 0:
                        #print(queue_name)
                        if queue_name == "depth":
                            print(len(packets))
                            depth_frame = packets[-1].getFrame()  # use latest packet
                            self.bus.publish("depth", depth_frame)

                        if queue_name == "color":
                            color_frame = packets[-1].getData().tobytes()  # use latest packet
                            self.bus.publish("color", color_frame)

                        if queue_name == "rotation":
                            #print(len(packets))
                            for packet in packets:
                                #print(len(packet.packets))
                                #for data in packet.packets:
                                #    rotdata = data.rotationVector
                                #    print(f"{rotdata.getTimestampDevice()}, {rotdata.i:.3f}, {rotdata.j:.3f}, {rotdata.k:.3f}, {rotdata.real:.3f}")

                                quaternions = [[data.rotationVector.getTimestampDevice().total_seconds(),  # timestamp
                                                data.rotationVector.i, data.rotationVector.j,
                                                data.rotationVector.k, data.rotationVector.real]
                                               for data in packet.packets]
                                self.bus.publish("rotation_list", quaternions)

    def request_stop(self):
        self.bus.sleep(1)
        self.bus.shutdown()
