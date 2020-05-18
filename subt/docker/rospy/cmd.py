
import time

import rospy
from rospy.impl import rosout
from std_srvs.srv import SetBool


def main():
    rospy.init_node('python3-test', log_level=rospy.DEBUG)
    print("Waiting for /rosout logger")
    while rosout._rosout_pub.get_num_connections() == 0:
        time.sleep(0.2)
    rospy.loginfo("python3 client running")

    params = rospy.get_param_names()
    for name in params:
        value = rospy.get_param(name)
        rospy.loginfo(f"{name}: {str(value)}")

    # we need these services
    rospy.loginfo("waiting for /subt/start and /subt/finish")
    rospy.wait_for_service('/subt/start')
    rospy.wait_for_service('/subt/finish')

    # if clock is not ticking, service /subt/start will return with error
    rospy.loginfo("waiting for clock")
    rospy.sleep(2)
    rospy.loginfo(repr(rospy.get_rostime()))

    try:
        subt_start = rospy.ServiceProxy('/subt/start', SetBool)
        ret = subt_start(True)
        rospy.loginfo("/subt/start: {0}".format(ret.success))

    finally:
        subt_finish = rospy.ServiceProxy('/subt/finish', SetBool)
        ret = subt_finish(True)
        rospy.loginfo("/subt/finish: {0}".format(ret.success))


if __name__ == "__main__":
    main()
