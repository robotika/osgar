#!/usr/bin/env python

import rospy
import tf
import geodesy.props
import geodesy.utm
import geodesy.wu_point
import geodesy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped


class GPSToUTC():
    def __init__(self):
        rospy.init_node('gps_to_UTC')
        self.trans = tf.TransformBroadcaster()
        self.x = 0
        self.y = 0
        self.gps_msg = None
        rospy.Subscriber('/ublox/fix', NavSatFix, callback=self.navsatfix_callback, queue_size=5)
        self.map_pose_pub = rospy.Publisher('/global_pose', PoseStamped, queue_size=5)
        self.rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.trans.sendTransform((self.x, self.y, 0), (0, 0, 0, 1), rospy.Time.now(), "base_link", "local_map")
            map_pose = PoseStamped()
            map_pose.header.stamp = rospy.Time.now()
            map_pose.header.frame_id = "local_map"
            map_pose.pose.position.x = self.x
            map_pose.pose.position.y = self.y
            self.map_pose_pub.publish(map_pose)
            print("map pose publish")
            self.rate.sleep()

    def navsatfix_callback(self, msg):
        if msg.status.status == -1:
            print("no GPS data received")
        else:
            point = geodesy.utm.fromLatLong(msg.latitude, msg.longitude).toPoint()
            self.x = float(point.x)
            self.y = float(point.y) 
            self.gps_msg = msg
                                                                
if __name__ == '__main__':
    try:
        GPSToUTC()
    except rospy.ROSInterruptException:
        pass

