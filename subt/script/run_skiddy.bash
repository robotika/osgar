#!/bin/bash

source ros/subt/setup.bash
export PYTHONPATH=${PWD}:${PYTHONPATH}
export OMP_NUM_THREADS=1  # Make OpenCV eat fewer cpus.
python3 -m osgar.record config/skiddy-subt-serialloop.json &
pidcore=$!
python3 -m subt.main --use-old-record run config/skiddy-subt.json --side right --speed 0.4 --timeout 900 --wall-dist 0.6 --gap-size 0.6 --init-path "15.0,0"
#python3 -m osgar.go run config/skiddy-subt.json -s 0.3 -d -4 --timeout 30
kill "$pidcore"