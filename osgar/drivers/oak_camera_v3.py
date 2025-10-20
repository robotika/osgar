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
                          'pose3d')
        self.fps = config.get('fps', 10)

        color_resolution_value = config.get("color_resolution", "THE_1080_P")
        self.color_resolution = g_resolution_dic[color_resolution_value]
        self.mono_resolution = config.get("mono_resolution", (640, 400))
        if isinstance(self.mono_resolution, str):
            assert self.mono_resolution in g_resolution_dic, self.mono_resolution
            self.mono_resolution = g_resolution_dic[self.mono_resolution]

        self.video_encoder = get_video_encoder(config.get('video_encoder', 'mjpeg'))
        self.video_encoder_h264_bitrate = config.get('h264_bitrate', 0)  # 0 = automatic

        self.is_depth = config.get('is_depth', False)

        self.sleep_on_start_sec = config.get('sleep_on_start_sec')
        self.verbose_detections = config.get('verbose_detections', True)

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

            def build(self, *args):
                self.link_args(*args)
                return self

            def process(self, frame):
                seq_num = frame.getSequenceNum()  # for sync of various outputs
                dt = frame.getTimestamp()  # datetime.timedelta
                timestamp_us = ((dt.days * 24 * 3600 + dt.seconds) * 1000000 + dt.microseconds)
                self.bus.publish("color_seq", [seq_num, timestamp_us])
                self.bus.publish("color", frame.getData().tobytes())


        with dai.Pipeline() as pipeline:
            # Define source and output
            cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

            # should frameRate = fps?? should this be limited on input?
            output = cam_rgb.requestOutput(self.color_resolution, type=dai.ImgFrame.Type.NV12, fps=self.fps)
            encoded = pipeline.create(dai.node.VideoEncoder).build(output,
                                                                   frameRate=self.fps,
                                                                   profile=self.video_encoder)
            saver = pipeline.create(VideoPublisher).build(encoded.out)
            saver.bus = self.bus

            if self.is_depth:
                mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
                mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
                stereo = pipeline.create(dai.node.StereoDepth)
                mono_left_out = mono_left.requestOutput(self.mono_resolution, fps=self.fps)
                mono_right_out = mono_right.requestOutput(self.mono_resolution, fps=self.fps)
                mono_left_out.link(stereo.left)
                mono_right_out.link(stereo.right)
                depth_queue = stereo.depth.createOutputQueue()

            # copy from basalt_vio.py
            imu = pipeline.create(dai.node.IMU)
            odom = pipeline.create(dai.node.BasaltVIO)

            imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
            imu.setBatchReportThreshold(1)
            imu.setMaxBatchReports(10)

            mono_left_out.link(odom.left)
            mono_right_out.link(odom.right)
            imu.out.link(odom.imu)
            odom_queue = odom.transform.createOutputQueue()

            # Connect to device and start pipeline
            pipeline.start()
            while pipeline.isRunning() and self.bus.is_alive():
                if self.is_depth:
                    depth_frame = depth_queue.get()
                    # TODO refactor, as this bit is the same as for "color_seq"
                    seq_num = depth_frame.getSequenceNum()  # for sync of various outputs
                    dt = depth_frame.getTimestamp()  # datetime.timedelta
                    timestamp_us = ((dt.days * 24 * 3600 + dt.seconds) * 1000000 + dt.microseconds)
                    self.bus.publish("depth_seq", [seq_num, timestamp_us])
                    self.bus.publish("depth", depth_frame.getCvFrame())

                    odom_frame = odom_queue.get()
                    t = odom_frame.getTranslation()
                    q = odom_frame.getQuaternion()
                    self.bus.publish("pose3d", [[t.x, t.y, t.z], [q.qx, q.qy, q.qz, q.qw]])
                else:
                    self.bus.sleep(0.1)

    def request_stop(self):
        self.bus.shutdown()
