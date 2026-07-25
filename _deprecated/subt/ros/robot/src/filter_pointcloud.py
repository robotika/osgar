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
    def __init__(self):
        self.points_subscriber = rospy.Subscriber("/points", PointCloud2, self.points_callback)
        self.points_publisher = rospy.Publisher("/points_cleaned", PointCloud2, queue_size=1)

        # Focal length.
        fx = rospy.get_param('~fx', 554.25469)
        # Image dimensions.
        self.camw = rospy.get_param('~image_width', 640)
        self.camh = rospy.get_param('~image_height', 480)
        # Principal point, position of optical axis
        self.rx = rospy.get_param('~principal_point_x', self.camw/2+0.5)
        self.ry = rospy.get_param('~principal_point_y', self.camh/2+0.5)
        # Distance reach of the RGBD camera range (meters)
        out_of_range = rospy.get_param('~max_range', 10) + 1.0

        # Pixel coordinates relative to the center of the image, with positive
        # directions to the left and up.
        pxs = self.rx - np.repeat(
                np.arange(self.camw).reshape((1, self.camw)), self.camh, axis=0)
        pys = self.ry - np.repeat(
                np.arange(self.camh).reshape((self.camh, 1)), self.camw, axis=1)
        pzs = np.ones((self.camh, self.camw), dtype=float)
        # For each pixel in the image, a vector representing its corresponding
        # direction in the scene with a unit forward axis.
        self.background = (np.dstack([pzs, pxs / fx, pys / fx]).T.reshape((3, -1))).reshape((3, self.camw, self.camh)).T * out_of_range

    def filter_points(self, msg):
        assert msg.height == self.camh, (msg.height, self.camh)
        assert msg.width == self.camw, (msg.width, self.camw)
        assert msg.point_step == 24, msg.point_step
        assert msg.row_step == self.camw * 24, (msg.row_step, self.camw * 24)

        data = np.frombuffer(msg.data, dtype=np.float32).reshape((self.camh, self.camw, 6))
        xyz = data[:, :, :3].copy()
        # convert +inf to real number outside sensor range
        mask = np.isposinf(xyz)
        xyz[:, :, :] = np.where(mask, self.background, xyz)

        # replace close reading (propellers) by -inf (blind zone)
        mask = (xyz[:, :, 0] < 0.32)
        mask = np.repeat(mask[:, :, np.newaxis], 3, axis=2)
        xyz[:, :, :] = np.where(mask, float('-inf'), xyz)
        new_data = xyz.tobytes()
        return new_data

    def points_callback(self, msg):
        new_data = self.filter_points(msg)
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
    pc_filter = FilterPointCloud()
    rospy.spin()

