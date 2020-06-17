#!/bin/bash
ssh -t k3jetson "source /opt/ros/melodic/setup.bash;source ~/ros_ws/devel/setup.bash; roslaunch robot k3.launch"
