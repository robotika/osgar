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

# this function is 1:1 to ver2
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

        self.video_encoder = get_video_encoder(config.get('video_encoder', 'mjpeg'))
        self.video_encoder_h264_bitrate = config.get('h264_bitrate', 0)  # 0 = automatic

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
            output = cam_rgb.requestOutput((1920, 1440), type=dai.ImgFrame.Type.NV12, fps=self.fps)
            encoded = pipeline.create(dai.node.VideoEncoder).build(output,
                                                                   frameRate=self.fps,
                                                                   profile=self.video_encoder)
            saver = pipeline.create(VideoPublisher).build(encoded.out)
            saver.bus = self.bus

            # Connect to device and start pipeline
            pipeline.start()
            while pipeline.isRunning() and self.bus.is_alive():
                self.bus.sleep(0.1)

    def request_stop(self):
        self.bus.shutdown()
