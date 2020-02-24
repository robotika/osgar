#!/bin/bash

#(python3 -m subt.main run config/maria-ros.json --side right --speed 0.5 --walldist 0.7 ;python3 -m osgar.record config/test-lora.json) &
#FIRST ROUND: (python3 -m subt.main run config/maria-ros.json --side left --speed 0.5 --walldist 0.7 --init-offset="-2.5,0,0" --start-paused;python3 -m osgar.record config/test-lora.json) &


(python3 -m subt.main run config/maria-ros.json --side left --speed 0.5 --walldist 0.6 --init-offset="-2.5,0,0" ;python3 -m osgar.record config/test-lora.json) &
roslaunch robot maria.launch

wait

