#!/usr/bin/python

import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
from laser_geometry import LaserProjection

class LaserScanToPointCloud:
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pointCloudPublisher = rospy.Publisher("/points", PointCloud2, queue_size = 1)
        self.laserScanSubscriber = rospy.Subscriber("/scan", LaserScan, self.laserScanCallback)

    def laserScanCallback(self, data):
        cloudOut = self.laserProj.projectLaser(data)
        self.pointCloudPublisher.publish(cloudOut)



if __name__ == "__main__":
    rospy.init_node("LaserScanToPointCloud")
    laserScanToPointCloud = LaserScanToPointCloud()
    rospy.spin()

