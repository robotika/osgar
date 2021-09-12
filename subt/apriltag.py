import sys
sys.path.append("/home/jakub/git/apriltag/build/")  # TODO integrate to system
import apriltag
import cv2
import numpy as np
from threading import Thread

from osgar.bus import BusShutdownException
from osgar.node import Node
from osgar.lib.depth import decompress as decompress_depth
from subt.artf_node import transform


class Apriltag(Node):

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("tag")

        self.thread = Thread(target=self.run)
        self.fx = config.get("fx")

    def start(self):
        self.thread.start()

    def join(self, timeout=None):
        self.thread.join(timeout=timeout)

    def run(self):
        detector = apriltag.apriltag('tag16h5', threads=1)
        try:
            while True:
                dt, channel, data = self.listen()
                if channel == "rgbd":
                    robot_pose, camera_pose, image_data, depth_data = data
                    img = np.frombuffer(image_data, dtype=np.uint8)
                    gray = cv2.imdecode(img, cv2.IMREAD_GRAYSCALE)
                    found = detector.detect(gray)
                    found = [tag for tag in found if tag['margin'] > 30 and tag['hamming'] == 0]
                    if len(found) > 0:
                        depth = decompress_depth(depth_data)
                        width = depth.shape[1]
                        height = depth.shape[0]
                        for tag in found:
                            tag_id = tag["id"]
                            cx, cy = tag["center"]
                            scale = depth[int(round(cy)), int(round(cx))]  # x coordinate, it corresponds with dist

                            camera_rel = [scale,  # relative X-coordinate in front
                                          scale * (width / 2 - cx) / self.fx,  # Y-coordinate is to the left
                                          scale * (height / 2 - cy) / self.fx]  # Z-up
                            # Coordinate of the apriltag relative to the robot.
                            robot_rel = transform(camera_pose, camera_rel)
                            # Global coordinate of the apriltag.
                            world_xyz = transform(robot_pose, robot_rel)
                            self.publish("tag", [tag_id, world_xyz])
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()
