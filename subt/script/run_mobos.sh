#!/bin/bash

#FIRST ROUND (python3 -m subt.main run config/mobos-ros.json --start-paused --side right --speed 0.5 --walldist 0.6 --init-offset="-2.5,0,0" --init-path="3, 0; 3, 3";python3 -m osgar.record config/test-lora.json) &

#SECOND ROUND (python3 -m subt.main run config/mobos-ros.json --side right --speed 0.5 --walldist 0.6 --init-offset="-2.5,0,0";python3 -m osgar.record config/test-lora.json) &

python3 -m subt.main run config/mobos-ros.json --side left --speed 0.5 --walldist 0.6 &
roslaunch robot auto_mob.launch

wait

