#!/bin/bash

trap "exit;" HUP INT TERM
trap "kill 0" EXIT
(python -m subt run config/kloubak2-subt-estop-lora-jetson.json --side left --speed 0.7 --timeout 1200 --start-paused; python -m osgar.record config/test-lora.json) &
ssh -t k2jetson "source /opt/ros/melodic/setup.bash;source ~/ros_ws/devel/setup.bash; roslaunch robot k2.launch"
wait
