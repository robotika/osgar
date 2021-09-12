#import sys
#sys.path.append("/home/jakub/git/apriltag/build/")  # TODO integrate to system
#import apriltag
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
        #detector = apriltag.apriltag('tag16h5', threads=1)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_16h5)
        parameters = cv2.aruco.DetectorParameters_create()
        try:
            while True:
                dt, channel, data = self.listen()
                if channel == "rgbd":
                    robot_pose, camera_pose, image_data, depth_data = data
                    img = np.frombuffer(image_data, dtype=np.uint8)
                    gray = cv2.imdecode(img, cv2.IMREAD_GRAYSCALE)
                    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

                    if ids is not None:
                        depth = decompress_depth(depth_data)
                        width = depth.shape[1]
                        height = depth.shape[0]
                        for tag_id, tag_corners in zip(ids.tolist(), corners):
                            cx = np.mean(tag_corners[0, :, 0])
                            cy = np.mean(tag_corners[0, :, 1])
                            scale = depth[int(round(cy)), int(round(cx))]  # x coordinate, it corresponds with dist
                            if scale < 0.5 or scale > 5:  # ignore too close and too far objects
                                continue

                            camera_rel = [scale,  # relative X-coordinate in front
                                          scale * (width / 2 - cx) / self.fx,  # Y-coordinate is to the left
                                          scale * (height / 2 - cy) / self.fx]  # Z-up
                            # Coordinate of the apriltag relative to the robot.
                            robot_rel = transform(camera_pose, camera_rel)
                            # Global coordinate of the apriltag.
                            world_xyz = transform(robot_pose, robot_rel)
                            #print([tag_id[0], world_xyz])
                            self.publish("tag", [tag_id[0], world_xyz])
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()
