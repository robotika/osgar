"""
  Wait for all necessary ROS sensors
"""
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry


sensors = set()


def callback_imu(data):
    global sensors
    if 'imu' not in sensors:
        rospy.loginfo(rospy.get_caller_id() + "Imu heard %s", data.data)
        sensors.add('imu')


def callback_scan(data):
    rospy.loginfo(rospy.get_caller_id() + "LidarScan heard %s", data.data)


def callback_image(data):
    rospy.loginfo(rospy.get_caller_id() + "Image heard %s", data.data)


def callback_odom(data):
    rospy.loginfo(rospy.get_caller_id() + "Odom heard %s", data.data)


def wait_for_sensors():
    rospy.init_node('listener', anonymous=True)
    rospy.loginfo('-------------- BEGIN --------------')
    rospy.Subscriber('/X2/imu/data', Imu, callback_imu)
    rospy.Subscriber('/X2/front_scan', LaserScan, callback_scan)
    rospy.Subscriber('/X2/front/image_raw/compressed', CompressedImage, callback_image)
    rospy.Subscriber('/X2/odom', Odometry, callback_odom)
    rospy.loginfo('--------------- END ---------------')
    rospy.spin()


if __name__ == '__main__':
    wait_for_sensors()

# vim: expandtab sw=4 ts=4
