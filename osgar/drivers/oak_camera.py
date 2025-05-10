"""
    Osgar driver for Luxonis OAK cameras.
    https://www.luxonis.com/
"""
from threading import Thread
from pathlib import Path
import logging

import depthai as dai
import numpy as np

g_logger = logging.getLogger(__name__)

g_resolution_dic = {
    "THE_400_P": (640, 400),
    "THE_480_P": (640, 480),
    "THE_720_P": (1280, 720),
    "THE_800_P": (1280, 800),
    "THE_1080_P": (1920, 1080),
    "THE_4_K": (3840, 2160),
    "THE_12_MP": (4000, 3000),
    "THE_13_MP": (4160, 3120)
}

def cam_is_available(cam_id):
    devices = dai.Device.getAllAvailableDevices()
    available_devices_names = []
    available_devices_MxId = []
    for dev in devices:
        if cam_id == dev.desc.name or cam_id == dev.getMxId():
            return True
        available_devices_names.append(dev.desc.name)
        available_devices_MxId.append(dev.getMxId())

    g_logger.error(f"{cam_id} was not found!")
    g_logger.info(f'Found device names: {", ".join(available_devices_names)}')
    g_logger.info(f'Found device MxIds: {", ".join(available_devices_MxId)}')

    return False


def get_video_encoder(name):
    # https://docs.luxonis.com/projects/api/en/latest/components/nodes/video_encoder/
    if name == 'h264':
        return dai.VideoEncoderProperties.Profile.H264_MAIN
    elif name == 'h265':
        return dai.VideoEncoderProperties.Profile.H265_MAIN
    elif name == 'mjpeg':
        return dai.VideoEncoderProperties.Profile.MJPEG
    else:
        assert 0, f'"{name}" is not supported'


class OakCamera:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus

        compress_depth_stream = config.get('compress_depth_stream', True)
        self.bus.register('depth:gz' if compress_depth_stream else 'depth',
                          'color', 'orientation_list', 'detections', 'left_im', 'right_im',
                          # *_seq streams are needed for output sync amd they are published BEFORE payload data
                          'depth_seq', 'color_seq', 'detections_seq', 'left_im_seq', 'right_im_seq',
                          'nn_mask:gz')
        self.fps = config.get('fps', 10)
        self.subsample = config.get('subsample')
        self.is_depth = config.get('is_depth', False)
        self.laser_projector_current = config.get("laser_projector_current", 0)
        assert self.laser_projector_current <= 1200, self.laser_projector_current  # The limit is 1200 mA.
        self.flood_light_current = config.get("flood_light_current", 0)
        assert self.flood_light_current <= 1500, self.flood_light_current  # The limit is 1500 mA.
        self.is_color = config.get('is_color', False)
        self.video_encoder = get_video_encoder(config.get('video_encoder', 'mjpeg'))
        self.video_encoder_h264_bitrate = config.get('h264_bitrate', 0)  # 0 = automatic
        self.is_stereo_images = config.get('is_stereo_images', False)

        self.is_imu_enabled = config.get('is_imu_enabled', False)
        # Preferred number of IMU records in one packet
        self.number_imu_records = config.get('number_imu_records', 20)
        self.disable_magnetometer_fusion = config.get('disable_magnetometer_fusion', False)
        self.cam_id = config.get('cam_id')  # https://docs.luxonis.com/software/depthai-components/device/

        self.is_extended_disparity = config.get("stereo_extended_disparity", False)
        self.is_subpixel = config.get("stereo_subpixel", False)
        self.is_left_right_check = config.get("stereo_left_right_check", False)
        assert not(self.is_extended_disparity and self.is_subpixel)  # Do not use extended_disparity and subpixel together.

        self.color_manual_focus = config.get("color_manual_focus")  # 0..255 [far..near]
        self.color_manual_exposure = config.get("color_manual_exposure")  # [exposure, iso] 1..33000 [us] and 100..1600
        self.color_manual_wb = config.get("color_manual_wb")  # 1000..12000 K
        self.stereo_manual_exposure = config.get("stereo_manual_exposure")  # [exposure, iso] 1..33000 [us] and 100..1600
        self.stereo_manual_wb = config.get("stereo_manual_wb")  # 1000..12000 K

        self.mono_resolution = config.get("mono_resolution", (640, 400))
        if isinstance(self.mono_resolution, str):
            assert self.mono_resolution in g_resolution_dic, self.mono_resolution
            self.mono_resolution = g_resolution_dic[self.mono_resolution]

        color_resolution_value = config.get("color_resolution", "THE_1080_P")
        self.color_resolution = getattr(dai.ColorCameraProperties.SensorResolution, color_resolution_value)

        color_orientation = config.get("color_orientation", "AUTO")
        assert color_orientation in ["AUTO", "HORIZONTAL_MIRROR", "NORMAL", "ROTATE_180_DEG", "VERTICAL_FLIP"], color_orientation
        self.color_orientation = getattr(dai.CameraImageOrientation, color_orientation)

        median_filter_value = config.get("stereo_median_filter", "KERNEL_7x7")
        assert median_filter_value in ["KERNEL_7x7", "KERNEL_5x5", "KERNEL_3x3", "MEDIAN_OFF"], median_filter_value
        self.median_filter = getattr(dai.MedianFilter, median_filter_value)

        stereo_mode_value = config.get("stereo_mode", "HIGH_DENSITY")
        assert stereo_mode_value in ["HIGH_DENSITY", "HIGH_ACCURACY"], stereo_mode_value
        self.stereo_mode = getattr(dai.node.StereoDepth.PresetMode, stereo_mode_value)

        self.alignment = config.get("color_depth_alignment", False)

        self.oak_config_model = config.get("model")
        self.oak_config_nn_config = config.get("nn_config", {})
        self.oak_config_nn_family = self.oak_config_nn_config.get("NN_family", "YOLO")
        self.labels = config.get("mappings", {}).get("labels", [])

        self.is_debug_mode = config.get('debug', False)  # run with debug output level

    def config_oak_nn(self, pipeline, camRgb):
        """
        Configure OAK pipeline for processing neural network
        """
        # code taken from
        # https://github.com/luxonis/depthai-experiments/blob/master/gen2-yolo/device-decoding/main_api.py
        nnConfig = self.oak_config_nn_config  # "nn_config" section

        # parse input shape
        if "input_size" in nnConfig:
            W, H = tuple(map(int, nnConfig.get("input_size").split('x')))

        nn_family = nnConfig.get("NN_family", "YOLO")

        # get model path
        nnPath = self.oak_config_model.get("blob")  # "model" section
        assert Path(nnPath).exists(), "No blob found at '{}'!".format(nnPath)

        if nn_family == 'YOLO':
            # extract metadata
            metadata = nnConfig.get("NN_specific_metadata", {})
            classes = metadata.get("classes", {})
            coordinates = metadata.get("coordinates", {})
            anchors = metadata.get("anchors", {})
            anchorMasks = metadata.get("anchor_masks", {})
            iouThreshold = metadata.get("iou_threshold", {})
            confidenceThreshold = metadata.get("confidence_threshold", {})

            print(metadata)


        # Define sources and outputs
        if nn_family == 'YOLO':
            detectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)
        else:
            detectionNetwork = pipeline.create(dai.node.NeuralNetwork)
        nnOut = pipeline.create(dai.node.XLinkOut)

        nnOut.setStreamName("nn")

        # Properties - NOTE - overwriting defaults from config!
        camRgb.setPreviewSize(W, H)

        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setFps(self.fps)

        # Network specific settings
        detectionNetwork.setBlobPath(nnPath)
        if nn_family == 'YOLO':
            detectionNetwork.setConfidenceThreshold(confidenceThreshold)
            detectionNetwork.setNumClasses(classes)
            detectionNetwork.setCoordinateSize(coordinates)
            detectionNetwork.setAnchors(anchors)
            detectionNetwork.setAnchorMasks(anchorMasks)
            detectionNetwork.setIouThreshold(iouThreshold)
            detectionNetwork.setNumInferenceThreads(2)
            detectionNetwork.input.setBlocking(False)

        # Linking
        camRgb.preview.link(detectionNetwork.input)
        detectionNetwork.out.link(nnOut.input)

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

            color.setImageOrientation(self.color_orientation)
            color.setResolution(self.color_resolution)
            color.setBoardSocket(dai.CameraBoardSocket.RGB)
            color.setFps(self.fps)
            # Value 130 for manual focus allows align color and depth frames.
            # https://docs.luxonis.com/projects/api/en/latest/samples/StereoDepth/rgb_depth_aligned/
            if self.color_manual_focus is not None:
                color.initialControl.setManualFocus(self.color_manual_focus)
            if self.color_manual_exposure is not None:
                exposure, iso = self.color_manual_exposure
                color.initialControl.setManualExposure(exposure, iso)  # exposure time and ISO

            if self.color_manual_wb is not None:
                color.initialControl.setManualWhiteBalance(self.color_manual_wb)

            color_encoder.setDefaultProfilePreset(self.fps, self.video_encoder)
            color_encoder.setBitrateKbps(self.video_encoder_h264_bitrate)

            color.video.link(color_encoder.input)
            color_encoder.bitstream.link(color_out.input)

        if self.is_depth or self.is_stereo_images:
            left = pipeline.create(dai.node.Camera)
            right = pipeline.create(dai.node.Camera)
            if self.is_depth:
                stereo = pipeline.create(dai.node.StereoDepth)
                depth_out = pipeline.create(dai.node.XLinkOut)
                queue_names.append("depth")
                depth_out.setStreamName("depth")

                stereo.setDefaultProfilePreset(self.stereo_mode)
                # https://docs.luxonis.com/projects/api/en/latest/components/nodes/stereo_depth/#currently-configurable-blocks
                stereo.initialConfig.setMedianFilter(self.median_filter)
                stereo.setExtendedDisparity(self.is_extended_disparity)
                stereo.setSubpixel(self.is_subpixel)
                # https://docs.luxonis.com/en/latest/pages/faq/#left-right-check-depth-mode
                stereo.setLeftRightCheck(self.is_left_right_check)
                if self.alignment and self.is_color:
                    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
                    # TODO check camera manual focus
                    g_logger.info("Alignment of depth to color is enabled."
                                  " Depth resolution will correspond with color camera.")

                # links
                left.video.link(stereo.left)
                right.video.link(stereo.right)
                stereo.depth.link(depth_out.input)

            if self.is_stereo_images:
                left_encoder = pipeline.create(dai.node.VideoEncoder)
                left_encoder.setDefaultProfilePreset(self.fps, self.video_encoder)
                left_encoder.setBitrateKbps(self.video_encoder_h264_bitrate)
                left_out = pipeline.create(dai.node.XLinkOut)

                right_encoder = pipeline.create(dai.node.VideoEncoder)
                right_encoder.setDefaultProfilePreset(self.fps, self.video_encoder)
                right_encoder.setBitrateKbps(self.video_encoder_h264_bitrate)
                right_out = pipeline.create(dai.node.XLinkOut)

                queue_names.append("left")
                queue_names.append("right")
                left_out.setStreamName("left")
                right_out.setStreamName("right")

                # links
                left.video.link(left_encoder.input)
                left_encoder.bitstream.link(left_out.input)

                right.video.link(right_encoder.input)
                right_encoder.bitstream.link(right_out.input)

            left.setSize(self.mono_resolution)
            left.setBoardSocket(dai.CameraBoardSocket.LEFT)
            left.setFps(self.fps)
            right.setSize(self.mono_resolution)
            right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
            right.setFps(self.fps)

            if self.stereo_manual_exposure is not None:
                exposure, iso = self.stereo_manual_exposure
                left.initialControl.setManualExposure(exposure, iso)  # exposure time and ISO
                right.initialControl.setManualExposure(exposure, iso)  # exposure time and ISO

            if self.stereo_manual_wb is not None:
                left.initialControl.setManualWhiteBalance(self.stereo_manual_wb)
                right.initialControl.setManualWhiteBalance(self.stereo_manual_wb)

        if self.is_imu_enabled:
            imu = pipeline.create(dai.node.IMU)
            imu_out = pipeline.create(dai.node.XLinkOut)

            imu_out.setStreamName("orientation")
            queue_names.append("orientation")

            if self.disable_magnetometer_fusion:
                imu.enableIMUSensor(dai.IMUSensor.GAME_ROTATION_VECTOR, 100)  # without magnetometer
            else:
                imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 100)

            imu.setBatchReportThreshold(self.number_imu_records)
            imu.setMaxBatchReports(20)
            imu.out.link(imu_out.input)

        if self.oak_config_model is not None:
            # configure OAK neural network
            assert self.is_color, "RGC camera must be enabled!"
            self.config_oak_nn(pipeline, color)  # note, that "color" is RGB camera
            queue_names.append("nn")

        if not queue_names:
            g_logger.error("No stream enabled!")
            self.request_stop()
            return

        if self.cam_id:
            if not cam_is_available(self.cam_id):
                self.request_stop()
                return
            device_info = dai.DeviceInfo(self.cam_id)
        else:
            device_info = None
            g_logger.info("Used the first available device.")

        # Connect to device and start pipeline
        with dai.Device(pipeline, device_info) as device:
            if self.is_debug_mode:
                # Set debugging level
                device.setLogLevel(dai.LogLevel.DEBUG)
                device.setLogOutputLevel(dai.LogLevel.DEBUG)

            if self.laser_projector_current:
                device.setIrLaserDotProjectorBrightness(self.laser_projector_current)
            if self.flood_light_current:
                device.setIrFloodLightBrightness(self.flood_light_current)
            while self.bus.is_alive():
                queue_events = device.getQueueEvents(queue_names)

                for queue_name in queue_events:
                    packets = device.getOutputQueue(queue_name).tryGetAll()
                    if len(packets) > 0:
                        seq_num = packets[-1].getSequenceNum()  # for sync of various outputs
                        if self.subsample and seq_num % self.subsample != 0:
                            continue
                        dt = packets[-1].getTimestamp()  # datetime.timedelta
                        timestamp_us = ((dt.days * 24 * 3600 + dt.seconds) * 1000000 + dt.microseconds)
                        if queue_name == "depth":
                            depth_frame = packets[-1].getFrame()  # use latest packet
                            self.bus.publish("depth_seq", [seq_num, timestamp_us])
                            self.bus.publish("depth", depth_frame)

                        if queue_name == "left":
                            left_frame = packets[-1].getData().tobytes()  # use latest packet
                            self.bus.publish("left_im_seq", [seq_num, timestamp_us])
                            self.bus.publish("left_im", left_frame)

                        if queue_name == "right":
                            right_frame = packets[-1].getData().tobytes()  # use latest packet
                            self.bus.publish("right_im_seq", [seq_num, timestamp_us])
                            self.bus.publish("right_im", right_frame)

                        if queue_name == "color":
                            color_frame = packets[-1].getData().tobytes()  # use latest packet
                            self.bus.publish("color_seq", [seq_num, timestamp_us])
                            self.bus.publish("color", color_frame)

                        if queue_name == "orientation":
                            for packet in packets:
                                quaternions = [[data.rotationVector.getTimestampDevice().total_seconds(),  # timestamp
                                                data.rotationVector.rotationVectorAccuracy,  # Accuracy in rad, zero for GAME_ROTATION_VECTOR ?
                                                data.rotationVector.i, data.rotationVector.j,
                                                data.rotationVector.k, data.rotationVector.real
                                                ]
                                               for data in packet.packets]
                                self.bus.publish("orientation_list", quaternions)

                        if queue_name == "nn":
                            if self.oak_config_nn_family == 'YOLO':
                                detections = packets[-1].detections
                                bbox_list = []
                                for detection in detections:
                                    bbox = (detection.xmin, detection.ymin, detection.xmax, detection.ymax)
                                    print(self.labels[detection.label], detection.confidence, bbox)
                                    bbox_list.append([self.labels[detection.label], detection.confidence, list(bbox)])
                                self.bus.publish("detections_seq", [seq_num, timestamp_us])
                                self.bus.publish('detections', bbox_list)
                            else:
                                nn_output = packets[-1].getLayerFp16('output')
                                WIDTH = 160
                                HEIGHT = 120
                                mask = np.array(nn_output).reshape((2, HEIGHT, WIDTH))
                                mask = mask.argmax(0).astype(np.uint8)
                                self.bus.publish('nn_mask', mask)

    def request_stop(self):
        self.bus.shutdown()
