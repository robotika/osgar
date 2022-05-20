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
        self.use_depth = config.get('depth', False)
        self.use_color = config.get('color', False)

        self.mono_resolution = dai.MonoCameraProperties.SensorResolution.THE_400_P
        self.color_resolution = dai.ColorCameraProperties.SensorResolution.THE_1080_P


    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        pipeline = dai.Pipeline()
        queue_names = []

        # Define sources and outputs, set properties and linking nodes.
        if self.use_color:
            color = pipeline.create(dai.node.ColorCamera)
            color_encoder = pipeline.create(dai.node.VideoEncoder)
            color_out = pipeline.create(dai.node.XLinkOut)

            queue_names.append("color")
            color_out.setStreamName('color')

            color.setResolution(self.color_resolution)
            color.setBoardSocket(dai.CameraBoardSocket.RGB)
            color.setFps(self.fps)
            color.initialControl.setManualFocus(130)
            color_encoder.setDefaultProfilePreset(self.fps, dai.VideoEncoderProperties.Profile.MJPEG)

            color.video.link(color_encoder.input)
            color_encoder.bitstream.link(color_out.input)

        if self.use_depth:
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

            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)  # TODO verify meaning
            stereo.setLeftRightCheck(True)  # https://docs.luxonis.com/en/latest/pages/faq/#left-right-check-depth-mode

            left.out.link(stereo.left)
            right.out.link(stereo.right)
            stereo.depth.link(depth_out.input)

        if not queue_names:  # No stream enabled
            return

        # TODO imu

        # Connect to device and start pipeline
        with dai.Device(pipeline) as device:
            while self.bus.is_alive():
                queue_events = device.getQueueEvents(queue_names)

                for queue_name in queue_events:
                    if queue_name == "depth":
                        packets = device.getOutputQueue(queue_name).tryGetAll()
                        if len(packets) > 0:
                            depth_frame = packets[-1].getFrame()  # use latest packet
                            self.bus.publish("depth", depth_frame)

                    if queue_name == "color":
                        packets = device.getOutputQueue(queue_name).tryGetAll()
                        if len(packets) > 0:
                            color_frame = packets[-1].getData().tobytes()  # use latest packet
                            self.bus.publish("color", color_frame)


    def request_stop(self):
        self.bus.shutdown()
