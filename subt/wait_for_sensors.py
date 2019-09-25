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
        rospy.loginfo(rospy.get_caller_id() + " Imu")
        sensors.add('imu')


def callback_scan(data):
    global sensors
    if 'scan' not in sensors:
        rospy.loginfo(rospy.get_caller_id() + " LidarScan")
        sensors.add('scan')


def callback_image(data):
    global sensors
    if 'image' not in sensors:
        rospy.loginfo(rospy.get_caller_id() + " Image")
        sensors.add('image')


def callback_odom(data):
    global sensors
    if 'odom' not in sensors:
        rospy.loginfo(rospy.get_caller_id() + " Odom")
        sensors.add('odom')


def wait_for_sensors():
    global sensors
    rospy.init_node('listener', anonymous=True)
    rospy.loginfo('-------------- BEGIN --------------')
    rospy.Subscriber('/X2/imu/data', Imu, callback_imu)
    rospy.Subscriber('/X2/front_scan', LaserScan, callback_scan)
    rospy.Subscriber('/X2/front/image_raw/compressed', CompressedImage, callback_image)
    rospy.Subscriber('/X2/odom', Odometry, callback_odom)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if len(sensors) >= 4:
            break
        r.sleep()
    rospy.loginfo('--------------- END ---------------')


if __name__ == '__main__':
    wait_for_sensors()

# vim: expandtab sw=4 ts=4
