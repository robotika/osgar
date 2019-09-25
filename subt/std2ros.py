"""
  Read stdin and send it to ROS
"""
import rospy
import sys


def std2ros():
    rospy.init_node('mdtalker', anonymous=True)
    for i, line in enumerate(sys.stdin):
        s = str(i) + ': ' + line.strip()
        rospy.loginfo(s)


if __name__ == '__main__':
    std2ros()

# vim: expandtab sw=4 ts=4
