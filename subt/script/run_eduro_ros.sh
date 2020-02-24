#!/bin/bash

trap "exit;" HUP INT TERM
trap "kill 0" EXIT
(python -m subt run config/eduro-subt-estop-lora-wifi-jetson.json --side right --speed 0.7 --walldist 0.6 --timeout 300 --start-paused; python -m osgar.record config/test-lora.json) &
ssh -t edurojetson "source /opt/ros/melodic/setup.bash;source ~/ros_ws/devel/setup.bash; roslaunch robot eduro.launch"
wait

