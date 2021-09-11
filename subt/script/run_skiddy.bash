#!/bin/bash

source ros/subt/setup.bash
export PYTHONPATH=${PWD}:${PYTHONPATH}
python3 -m subt --use-old-record run config/skiddy-subt.json --side left --speed 0.4 --timeout 600 --wall-dist 1.0 --gap-size 1.0
