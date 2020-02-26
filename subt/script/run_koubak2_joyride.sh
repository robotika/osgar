#!/bin/bash

trap "exit;" HUP INT TERM
trap "kill 0" EXIT
python -m osgar.record config/kloubak2-joyride.json &
ssh -t k2jetson "source /opt/ros/melodic/setup.bash;source ~/ros_ws/devel/setup.bash; roslaunch robot k2_joyride.launch"
wait
