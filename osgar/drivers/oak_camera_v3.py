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
        with dai.Pipeline() as pipeline:

            # Define source and output
            cam = pipeline.create(dai.node.Camera).build()
            videoQueue = cam.requestOutput((640,400)).createOutputQueue()

            # Connect to device and start pipeline
            pipeline.start()
            while pipeline.isRunning() and self.bus.is_alive():
                videoIn = videoQueue.get()
                assert isinstance(videoIn, dai.ImgFrame)
                success, encoded_image = cv2.imencode('*.jpeg', videoIn.getCvFrame())
                if success:
                    color_frame = encoded_image.tobytes()
                    self.bus.publish("color", color_frame)

    def request_stop(self):
        self.bus.shutdown()
