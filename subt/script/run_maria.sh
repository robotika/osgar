#!/bin/bash

(python3 -m subt.main run config/maria-ros.json --side left --speed 0.5 --walldist 0.7 --start-paused;python3 -m osgar.record config/test-lora.json) &
roslaunch robot maria.launch

wait

