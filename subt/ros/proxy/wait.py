#!/usr/bin/env python2

import errno
import time
import socket

import rospy
import rostopic


def wait_for_master():
    """Return when /use_sim_time is set in rosparams. If rospy.init_node is called before /use_sim_time
    is set, it uses real time."""
    print("waiting for rosmaster...")
    last_params = []
    while not rospy.is_shutdown():
        try:
            params = rospy.get_param_names()
            if params != last_params:
                if '/use_sim_time' in params:
                    print("rosmaster ready")
                    return True
                last_params = params
            time.sleep(0.1)
        except socket.error as serr:
            if serr.errno != errno.ECONNREFUSED:
                raise serr  # re-raise error if its not the one we want
            time.sleep(0.5)


def wait_for_bridge(robot_name):
    prefix = "/{}/".format(robot_name)
    last_sub = set([])
    last_pub = set([])
    same_num = 0
    while True:
        publishers, subscribers = rostopic.get_topic_list()
        names_sub = set([a[0] for a in subscribers if a[0].startswith(prefix)])
        names_pub = set([a[0] for a in publishers if a[0].startswith(prefix)])
        status = "{} subscribers, {} publishers".format(len(names_sub), len(names_pub))
        if names_sub == last_sub and names_pub == last_pub:
            if same_num >= 3:
                rospy.loginfo("bridge ready: " + status)
                break
            else:
                same_num += 1
        else:
            same_num = 0
        rospy.loginfo("waiting for bridge "+prefix+"... " + status)
        rospy.sleep(1)
        last_sub = names_sub
        last_pub = names_pub


def has_breadcrumbs(robot_name):
    prefix = "/{}/".format(robot_name)
    breadcrumbs = prefix + "breadcrumb/deploy"
    publishers, subscribers = rostopic.get_topic_list()
    for name, _, _ in subscribers:
        if name == breadcrumbs:
            return True
    return False


def detect_config(robot_name):
    robot_description = rospy.get_param("/{}/robot_description".format(robot_name))
    if "robotika_x2_sensor_config_1" in robot_description:
        return "ROBOTIKA_X2_SENSOR_CONFIG_1"
    elif "ssci_x4_sensor_config_2" in robot_description:
        return "SSCI_X4_SENSOR_CONFIG_2"
    elif "TeamBase" in robot_description:
        return "TEAMBASE"
    elif "robotika_freyja_sensor_config" in robot_description:
        if has_breadcrumbs(robot_name):
            return "ROBOTIKA_FREYJA_SENSOR_CONFIG_2"
        return "ROBOTIKA_FREYJA_SENSOR_CONFIG_1"
    elif "robotika_kloubak_sensor_config" in robot_description:
        if has_breadcrumbs(robot_name):
            return "ROBOTIKA_KLOUBAK_SENSOR_CONFIG_2"
        return "ROBOTIKA_KLOUBAK_SENSOR_CONFIG_1"
    elif "explorer_r2_sensor_config" in robot_description:
        if has_breadcrumbs(robot_name):
            return "EXPLORER_R2_SENSOR_CONFIG_2"
        return "EXPLORER_R2_SENSOR_CONFIG_1"
    else:
        rospy.logerr("unknown configuration")
        return


def main():
    wait_for_master()
    rospy.init_node('wait', log_level=rospy.DEBUG)
    robot_name = rospy.get_param("/robot_names")
    wait_for_bridge(robot_name)
    robot_config = detect_config(robot_name)
    rospy.set_param("/robot_config", robot_config)


if __name__ == '__main__':
    main()
