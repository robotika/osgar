#!/bin/bash
ssh -t k2jetson "source /opt/ros/melodic/setup.bash;source ~/ros_ws/devel/setup.bash; roslaunch robot k2.launch"
