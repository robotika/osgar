#!/bin/bash

#trap "exit;" HUP INT TERM
#trap "kill 0" EXIT

# when signal received just exit
trap "echo 'got signal... exiting'; exit;" HUP INT TERM

# when exiting, send SIGINT to all processes in current process group
# and wait for them to finish
trap "echo 'exiting... killing children...';  kill -s SIGINT 0; wait" EXIT

(python -m subt --use-old-record run config/kloubak2-subt-estop-lora-jetson.json --side left --speed 0.5 --timeout 600; python -m osgar.record config/test-lora.json) &
ssh -t k2jetson_front "python3 -m osgar.record ~/git/osgar/config/jetson-node-k2-front.json" &
ssh -t k2jetson_rear "python3 -m osgar.record ~/git/osgar/config/jetson-node-k2-rear.json"

wait
