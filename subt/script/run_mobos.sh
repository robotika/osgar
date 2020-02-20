#!/bin/bash

(python3 -m subt.main run config/mobos-ros.json --side left --speed 0.5 --walldist 0.5 --start-stopped;python3 -m osgar.record config/test-lora.json) &
#python3 -m subt.main run config/mobos-ros.json --side left --speed 0.5 --walldist 0.5 &
roslaunch robot auto_mob.launch

wait

