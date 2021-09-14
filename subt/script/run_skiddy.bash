#!/bin/bash

source ros/subt/setup.bash
export PYTHONPATH=${PWD}:${PYTHONPATH}
export OMP_NUM_THREADS=1  # Make OpenCV eat fewer cpus.
python3 -m subt.main --use-old-record run config/skiddy-subt.json --side left --speed 0.4 --timeout 180 --wall-dist 1.0 --gap-size 1.0
#python3 -m osgar.go run config/skiddy-subt.json -s 0.3 -d -4 --timeout 30
