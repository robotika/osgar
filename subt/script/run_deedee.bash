#!/bin/bash

source ros/subt/setup.bash
export PYTHONPATH=${PWD}:${PYTHONPATH}
python3 -m subt --use-old-record run config/deedee-subt.json --side left --speed 0.2 --timeout 1200 --wall-dist 0.5 --gap-size 0.6
