#!/bin/bash
trap "exit;" HUP INT TERM
trap "kill 0" EXIT
ssh -t k3jetson "source /opt/ros/melodic/setup.bash;source ~/ros_ws/devel/setup.bash; roslaunch robot k3.launch"
