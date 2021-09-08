#!/bin/bash

source ros/subt/setup.bash
python3 -m subt --use-old-record run config/deedee-subt.json --side left --speed 0.05 --timeout 1200 --wall-dist 0.5 --gap-size 0.6
