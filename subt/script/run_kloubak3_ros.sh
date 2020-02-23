#!/bin/bash

trap "exit;" HUP INT TERM
trap "kill 0" EXIT
(python -m subt run config/kloubak3-subt-estop-lora-jetson.json --side left --speed 0.7 --timeout 120 --walldist 0.6; python -m osgar.record config/test-lora.json) &
ssh -t k3jetson "source /opt/ros/melodic/setup.bash;source ~/ros_ws/devel/setup.bash; roslaunch robot k3.launch"
wait
