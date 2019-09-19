#!/usr/bin/env bash

. /opt/ros/melodic/setup.bash
. ~/subt_solution/install/setup.sh

# Wait for the bridge
sleep 30

# Run your solution.
#roslaunch subt_seed x1.launch

roslaunch subt_example example_robot.launch &

#export PYTHONPATH=${PYTHONPATH}:$(pwd)/osgar
cd osgar

python3 -m subt run subt/subt-x2.json --side left --walldist 2.0 --timeout 180 --note "try to visit artifact and return home"

