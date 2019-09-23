#!/usr/bin/env bash

. /opt/ros/melodic/setup.bash
. ~/subt_solution/install/setup.sh

# Wait for the bridge
sleep 30

# Run your solution.
roslaunch subt_seed x1.launch &
sleep 10

cd osgar

python3 -m subt run subt/subt-x2.json --side left --walldist 0.75 --timeout 180 --note "try to visit artifact and return home"

