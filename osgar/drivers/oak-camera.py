"""
    Driver TODO
"""

from threading import Thread
# import numpy as np

import depthai as dai


class OakCamera:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus

        self.bus.register('depth', 'color', 'rotation')
        self.fps = config.get('fps', 10)

        self.mono_resolution = dai.MonoCameraProperties.SensorResolution.THE_400_P
        # self.color_resolution = dai.ColorCameraProperties.SensorResolution.THE_1080_P


    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        pipeline = dai.Pipeline()
        queue_names = []

        # Define sources and outputs
        # color = pipeline.create(dai.node.ColorCamera)
        left = pipeline.create(dai.node.MonoCamera)
        right = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        # TODO imu
        # color_out = pipeline.create(dai.node.XLinkOut)
        depth_out = pipeline.create(dai.node.XLinkOut)

        # color_out.setStreamName("color")
        # queue_names.append("color")
        depth_out.setStreamName("depth")
        queue_names.append("dept")

        # Set properties
        left.setResolution(self.mono_resolution)
        left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        left.setFps(self.fps)
        right.setResolution(self.mono_resolution)
        right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        right.setFps(self.fps)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)  # TODO verify meaning
        stereo.setLeftRightCheck(True)

        # Linking
        left.out.link(stereo.left)
        right.out.link(stereo.right)
        stereo.depth.link(depth_out.input)

        # Connect to device and start pipeline
        with dai.Device(pipeline) as device:
            while self.bus.is_alive():
                queue_events = device.getQueueEvents(("depth"))

                for queue_name in queue_events:
                    if queue_name == "depth":
                        packets = device.getOutputQueue(queue_name).tryGetAll()
                        if len(packets) > 0:
                            depth_frame = packets[-1].getFrame()  # use latest packet
                            self.bus.publish("depth", depth_frame)


    def request_stop(self):
        self.bus.shutdown()
