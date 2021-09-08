#!/bin/bash

trap "exit;" HUP INT TERM
trap "kill 0" EXIT
(python -m subt run config/kloubak2-subt-estop-lora-jetson.json --side left --speed 0.5 --timeout 600 --use-old-record; python -m osgar.record config/test-lora.json) &
ssh -t k2jetson_front   "cd ~/ros_ws/src/osgar/; python3 -m osgar.record ~/ros_ws/src/osgar/config/jetson-node-k2-front.json"
wait
