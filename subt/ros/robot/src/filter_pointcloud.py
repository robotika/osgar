#!/usr/bin/python
"""
  Filter PointCloud2 received from X4 drone for Octomap server
  - remove nearby propellers (up to 31cm) in the RGBD camera view
  - replace xyz = (inf, inf, inf) to range larger than 10m
"""
# http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html
# http://docs.ros.org/en/api/sensor_msgs/html/point__cloud2_8py_source.html
import ctypes
import struct

import rospy
from sensor_msgs.msg import PointCloud2, PointField

import numpy as np


class FilterPointCloud:
    def __init__(self,
                 # Focal length.
                 fx=554.25469,
                 # Image dimensions.
                 image_size=(640, 480),
                 # Principal point, position of optical axis
                 principal_point=(640 / 2. + 0.5, 480 / 2 + 0.5),
                 # Distance outside RGBD camera range (meters)
                 out_of_range=11.0,
                 ):
        self.points_subscriber = rospy.Subscriber("/points", PointCloud2, self.points_callback)
        self.points_publisher = rospy.Publisher("/points_cleaned", PointCloud2, queue_size=1)

        self.fx = fx
        self.camw, self.camh = image_size
        self.rx, self.ry = principal_point

        # Pixel coordinates relative to the center of the image, with positive
        # directions to the left and up.
        pxs = self.rx - np.repeat(
                np.arange(self.camw).reshape((1, self.camw)), self.camh, axis=0)
        pys = self.ry - np.repeat(
                np.arange(self.camh).reshape((self.camh, 1)), self.camw, axis=1)
        pzs = np.ones((self.camh, self.camw), dtype=np.float) * out_of_range
        # For each pixel in the image, a vector representing its corresponding
        # direction in the scene with a unit forward axis.
        self.background = (np.dstack([pzs, pxs / fx, pys / fx]).T.reshape((3, -1))).reshape((3, self.camw, self.camh)).T

    def points_callback(self, msg):
        assert msg.height == 480, msg.height
        assert msg.width == 640, msg.width
        assert msg.point_step == 24, msg.point_step
        assert msg.row_step == 640 * 24, msg.row_step

        data = np.frombuffer(msg.data, dtype=np.float32).reshape((480, 640, 6))
        xyz = data[:, :, :3]
        # convert +inf to real number outside sensor range
        mask = np.isposinf(xyz[:, :, 0])
        xyz[:, :, :] = np.where(mask, background, xyz)

        # replace close reading (propellers) by -inf (blind zone)
        mask = (xyz[:, :, 0] < 0.32)
        xyz[:, :, :] = np.where(mask, float('-inf'), xyz)
        new_data = data.tobytes()

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)]

        cloud_struct = struct.Struct('<fff')
        new_msg = PointCloud2(header=msg.header,
                              height=msg.height,
                              width=msg.width,
                              is_dense=False,
                              is_bigendian=False,
                              fields=fields,
                              point_step=cloud_struct.size,
                              row_step=cloud_struct.size * msg.width,
                              data=new_data)
        self.points_publisher.publish(new_msg)


if __name__ == "__main__":
    rospy.init_node("FilterPointCloud")
    filter = FilterPointCloud()
    rospy.spin()

