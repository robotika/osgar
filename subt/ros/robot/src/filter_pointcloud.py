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
#        self.points_subscriber = rospy.Subscriber("/input_points", PointCloud2, self.points_callback)
        self.points_subscriber = rospy.Subscriber("/A100LXM/rgbd_camera/depth/points", PointCloud2, self.points_callback)
#        self.points_publisher = rospy.Publisher("/output_points", PointCloud2, queue_size = 1)
        self.points_publisher = rospy.Publisher("/points_cleaned", PointCloud2, queue_size=1)

        # create flat "infinity" background
        size = 640 * 480
        self.inf_x = np.zeros(size, dtype=np.float32)
        self.inf_y = np.zeros(size, dtype=np.float32)
        self.inf_z = np.zeros(size, dtype=np.float32)
        fx = 554.25469
        MAX_VALUE = 11.0
        for x in range(640):
            for y in range(480):
                pos = y * 640 + x
                self.inf_x[pos] = MAX_VALUE
                self.inf_y[pos] = MAX_VALUE * (320.5 - x)/fx
                self.inf_z[pos] = MAX_VALUE * (240.5 - y)/fx

    def points_callback(self, msg):
        assert msg.height == 480, msg.height
        assert msg.width == 640, msg.width
        assert msg.point_step == 24, msg.point_step
        assert msg.row_step == 640 * 24, msg.row_step
        arr = np.frombuffer(msg.data, dtype=np.float32)
        x = arr[::6].copy()
        y = arr[1::6].copy()
        z = arr[2::6].copy()

        # limit infinite readings to 11 meters (out of range)
        mask = x == float('inf')
        x[mask] = self.inf_x[mask]
        y[mask] = self.inf_y[mask]
        z[mask] = self.inf_z[mask]

        mask = x < 0.31  # propellers
        x[mask] = float('-inf')
        y[mask] = float('-inf')
        z[mask] = float('-inf')

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)]

        cloud_struct = struct.Struct('<fff')
        buff = ctypes.create_string_buffer(cloud_struct.size * len(x))

        # TODO faster conversion
        bytes_arr = np.vstack([x, y, z]).flatten('f').tobytes()
        for i in range(len(bytes_arr)):
            buff[i] = bytes_arr[i]

        new_msg = PointCloud2(header=msg.header,
                              height=msg.height,
                              width=msg.width,
                              is_dense=False,
                              is_bigendian=False,
                              fields=fields,
                              point_step=cloud_struct.size,
                              row_step=cloud_struct.size * msg.width,
                              data=buff.raw)

        self.points_publisher.publish(new_msg)


if __name__ == "__main__":
    rospy.init_node("FilterPointCloud")
    filter = FilterPointCloud()
    rospy.spin()

