"""
    Osgar driver for Luxonis OAK cameras. (depthai v3)
    https://www.luxonis.com/
"""
from threading import Thread
from pathlib import Path
import logging

import depthai as dai
import numpy as np
import cv2

# ----------- oak_camera_v2.py copy & paste ----------------
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
# ----------- END OF oak_camera_v2.py copy & paste ----------------

class OakCamera:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus

        compress_depth_stream = config.get('compress_depth_stream', True)
        self.bus.register('depth:gz' if compress_depth_stream else 'depth',
                          'color', 'orientation_list', 'detections', 'left_im', 'right_im',
                          # *_seq streams are needed for output sync amd they are published BEFORE payload data
                          'depth_seq', 'color_seq', 'detections_seq', 'left_im_seq', 'right_im_seq',
                          'nn_mask:gz',
                          'pose3d', 'gridmap',
                          'redroad:gz', 'robotourist:gz')

        self.is_color = config.get('is_color', False)
        self.is_depth = config.get('is_depth', False)
        self.is_stereo_images = config.get('is_stereo_images', False)
        self.fps = config.get('fps', 10)

        color_resolution_value = config.get("color_resolution", "THE_1080_P")
        self.color_resolution = g_resolution_dic[color_resolution_value]
        self.mono_resolution = config.get("mono_resolution", (640, 400))
        if isinstance(self.mono_resolution, str):
            assert self.mono_resolution in g_resolution_dic, self.mono_resolution
            self.mono_resolution = g_resolution_dic[self.mono_resolution]

        self.video_encoder = get_video_encoder(config.get('video_encoder', 'mjpeg'))
        self.video_encoder_h264_bitrate = config.get('h264_bitrate', 0)  # 0 = automatic

        self.is_slam = config.get('is_slam', False)
        self.is_visual_odom = config.get('is_visual_odom', False)

        self.sleep_on_start_sec = config.get('sleep_on_start_sec')
        self.verbose_detections = config.get('verbose_detections', True)

        # NN setup variables - support for multiple models
        self.models = config.get("models", [])

        # Backwards compatibility for single-model configs
        if "model" in config and len(self.models) == 0:
            self.models.append({
                "model": config.get("model"),
                "nn_config": config.get("nn_config", {}),
                "mappings": config.get("mappings", {})
            })

    def start(self):
        if self.sleep_on_start_sec is not None:
            print(f'sleeping for {self.sleep_on_start_sec}s')
            self.bus.sleep(self.sleep_on_start_sec)
            print(f'END of sleep, starting ...')
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):

        class VideoPublisher(dai.node.HostNode):
            def __init__(self, *args, **kwargs):
                dai.node.HostNode.__init__(self, *args, **kwargs)
                self.bus = None
                self.stream_type = None

            def build(self, *args):
                self.link_args(*args)
                return self

            def process(self, frame):
                seq_num = frame.getSequenceNum()  # for sync of various outputs
                dt = frame.getTimestamp()  # datetime.timedelta
                timestamp_us = ((dt.days * 24 * 3600 + dt.seconds) * 1000000 + dt.microseconds)
                self.bus.publish(f"{self.stream_type}_seq", [seq_num, timestamp_us])
                self.bus.publish(self.stream_type, frame.getData().tobytes())


        with dai.Pipeline() as pipeline:
            # Define source and output
            if self.is_color:
                cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

                # should frameRate = fps?? should this be limited on input?
                output = cam_rgb.requestOutput(self.color_resolution, type=dai.ImgFrame.Type.NV12, fps=self.fps)
                encoded = pipeline.create(dai.node.VideoEncoder).build(output,
                                                                       frameRate=self.fps,
                                                                       profile=self.video_encoder)
                saver = pipeline.create(VideoPublisher).build(encoded.out)
                saver.bus = self.bus
                saver.stream_type = "color"


            if self.is_depth or self.is_stereo_images:
                mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
                mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

                # type=dai.ImgFrame.Type.NV12 ??
                mono_left_out = mono_left.requestOutput(self.mono_resolution, type=dai.ImgFrame.Type.NV12, fps=self.fps)
                mono_right_out = mono_right.requestOutput(self.mono_resolution, type=dai.ImgFrame.Type.NV12, fps=self.fps)
                if self.is_depth:
                    stereo = pipeline.create(dai.node.StereoDepth)
                    mono_left_out.link(stereo.left)
                    mono_right_out.link(stereo.right)
                    depth_queue = stereo.depth.createOutputQueue(blocking=False)

                if self.is_stereo_images:
                    for mono_out, stream_type in zip([mono_left_out, mono_right_out], ['left_im', 'right_im']):
                        mono_encoded = pipeline.create(dai.node.VideoEncoder).build(mono_out,
                                                                                    frameRate=self.fps,
                                                                                    profile=self.video_encoder)
                        mono_saver = pipeline.create(VideoPublisher).build(mono_encoded.out)
                        mono_saver.bus = self.bus
                        mono_saver.stream_type = stream_type

            # copy from basalt_vio.py
            imu = pipeline.create(dai.node.IMU)
            if self.is_visual_odom:
                odom = pipeline.create(dai.node.BasaltVIO)

            imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
            imu.setBatchReportThreshold(1)
            imu.setMaxBatchReports(10)

            if self.is_visual_odom:
                imu.out.link(odom.imu)
                odom_queue = odom.transform.createOutputQueue(blocking=False)

            if self.is_slam:
                slam = pipeline.create(dai.node.RTABMapSLAM)
                params = {"RGBD/CreateOccupancyGrid": "true",
                          "Grid/3D": "true",
                          "Rtabmap/SaveWMState": "true"}
                slam.setParams(params)

            if self.is_depth:
                stereo.setExtendedDisparity(False)
                stereo.setLeftRightCheck(True)
                stereo.setSubpixel(True)
                stereo.setRectifyEdgeFillColor(0)
                stereo.enableDistortionCorrection(True)
                stereo.initialConfig.setLeftRightCheckThreshold(10)
                stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)

                if self.is_visual_odom:
                    stereo.syncedLeft.link(odom.left)
                    stereo.syncedRight.link(odom.right)

            if self.is_slam:
                stereo.depth.link(slam.depth)
                stereo.rectifiedLeft.link(slam.rect)
                odom.transform.link(slam.odom)
                gridmap_queue = slam.occupancyGridMap.createOutputQueue(blocking=False)

            # Configure Neural Network processing blocks dynamically
            nn_queues = []
            for model_cfg in self.models:
                nn_path = model_cfg.get("model", {}).get("blob")
                if not nn_path:
                    continue
                assert Path(nn_path).exists(), "No blob found at '{}'!".format(nn_path)

                nn_config = model_cfg.get("nn_config", {})
                nn_family = nn_config.get("NN_family", "YOLO")

                input_size_str = nn_config.get("input_size")
                if input_size_str:
                    W, H = tuple(map(int, input_size_str.split('x')))
                else:
                    W, H = 416, 416  # default fallback

                nn = pipeline.create(dai.node.NeuralNetwork)
                nn.setBlobPath(nn_path)
                nn.setNumInferenceThreads(2)
                nn.input.setBlocking(False)

                if nn_family == 'YOLO':
                    metadata = nn_config.get("NN_specific_metadata", {})
                    parser = pipeline.create(dai.node.DetectionParser)
                    if "confidence_threshold" in metadata:
                        parser.setConfidenceThreshold(metadata["confidence_threshold"])
                    if "classes" in metadata:
                        parser.setNumClasses(metadata["classes"])
                    if "coordinates" in metadata:
                        parser.setCoordinateSize(metadata["coordinates"])
                    if "anchors" in metadata:
                        parser.setAnchors(metadata["anchors"])
                    if "anchor_masks" in metadata:
                        parser.setAnchorMasks(metadata["anchor_masks"])
                    if "iou_threshold" in metadata:
                        parser.setIouThreshold(metadata["iou_threshold"])

                    nn.out.link(parser.input)
                    queue = parser.out.createOutputQueue(blocking=False)
                else:
                    queue = nn.out.createOutputQueue(blocking=False)

                # Feed from color camera via specific requested output resolution per model
                nn_cam_out = cam_rgb.requestOutput((W, H), type=dai.ImgFrame.Type.BGR888p)
                nn_cam_out.link(nn.input)

                nn_queues.append({
                    "queue": queue,
                    "family": nn_family,
                    "image_size": (W, H),
                    "labels": model_cfg.get("mappings", {}).get("labels", [])
                })


            # Connect to device and start pipeline
            pipeline.start()

            while pipeline.isRunning() and self.bus.is_alive():
                processed_any = False

                # 1. Check Neural Networks
                for nn_setup in nn_queues:
                    queue = nn_setup["queue"]
                    nn_family = nn_setup["family"]
                    W, H = nn_setup["image_size"]
                    labels = nn_setup["labels"]

                    nn_packets = queue.tryGetAll()
                    if nn_packets and len(nn_packets) > 0:
                        processed_any = True
                        packet = nn_packets[-1]  # use latest packet
                        seq_num = packet.getSequenceNum()
                        dt = packet.getTimestamp()
                        timestamp_us = ((dt.days * 24 * 3600 + dt.seconds) * 1000000 + dt.microseconds)

                        if nn_family == 'YOLO':
                            detections = packet.detections
                            bbox_list = []
                            for detection in detections:
                                bbox = (detection.xmin, detection.ymin, detection.xmax, detection.ymax)
                                if self.verbose_detections:
                                    print(labels[detection.label], detection.confidence, bbox)
                                bbox_list.append([labels[detection.label], detection.confidence, list(bbox)])
                            self.bus.publish("detections_seq", [seq_num, timestamp_us])
                            self.bus.publish('detections', bbox_list)

                        elif nn_family == 'resnet':
                            nn_output = packet.getLayerFp16('output')
                            mask = np.array(nn_output).reshape((2, H, W))
                            mask = mask.argmax(0).astype(np.uint8)
                            self.bus.publish('nn_mask', mask)

                        elif nn_family == 'robotourist' or nn_family == 'redroad':
                            # v3: getTensor automatically returns a NumPy array
                            nn_output = packet.getTensor('redroad_output')
                            nn_robotourist_output = packet.getTensor('robotourist_output')

                            redroad = np.array(nn_output).reshape((H//2, W//2))
                            self.bus.publish('redroad', redroad)
                            mask = (redroad > 0).astype(np.uint8)
                            mask = np.array(mask).reshape((H//2, W//2))
                            self.bus.publish('nn_mask', mask)

                            # ver0 (1280, 7, 7), ver1 (160, 7, 7)
                            robotourist = np.array(nn_robotourist_output).reshape((len(nn_robotourist_output), 7, 7))
                            self.bus.publish('robotourist', robotourist)


                # 2. Check Depth
                if self.is_depth:
                    depth_frames = depth_queue.tryGetAll()
                    if depth_frames and len(depth_frames) > 0:
                        processed_any = True
                        depth_frame = depth_frames[-1]
                        seq_num = depth_frame.getSequenceNum()
                        dt = depth_frame.getTimestamp()
                        timestamp_us = ((dt.days * 24 * 3600 + dt.seconds) * 1000000 + dt.microseconds)
                        self.bus.publish("depth_seq", [seq_num, timestamp_us])
                        self.bus.publish("depth", depth_frame.getCvFrame())

                # 3. Check Visual Odom
                if self.is_visual_odom:
                    odom_frames = odom_queue.tryGetAll()
                    if odom_frames and len(odom_frames) > 0:
                        processed_any = True
                        odom_frame = odom_frames[-1]
                        t = odom_frame.getTranslation()
                        q = odom_frame.getQuaternion()
                        self.bus.publish("pose3d", [[t.x, t.y, t.z], [q.qx, q.qy, q.qz, q.qw]])

                # 4. Check SLAM
                if self.is_slam:
                    gridmaps = gridmap_queue.tryGetAll()
                    if gridmaps and len(gridmaps) > 0:
                        processed_any = True
                        gridmap = gridmaps[-1]
                        self.bus.publish('gridmap', gridmap.getFrame())

                # Only rest the CPU if no frames were pulled in this tick loop
                if not processed_any:
                    self.bus.sleep(0.01)

    def request_stop(self):
        self.bus.shutdown()
