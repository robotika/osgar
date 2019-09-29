#!/usr/bin/env bash

. /opt/ros/melodic/setup.bash
. ~/subt_solution/install/setup.sh

# Wait for the bridge
sleep 30

cd osgar
python3 -m osgar.record config/zeromq-fwd.json &
cd ..

# Run your solution.
#roslaunch subt_seed x1.launch &
roslaunch subt_seed x1.launch

#sleep 10

#cd osgar

#python subt/wait_for_sensors.py

#python3 -m subt run subt/subt-x2.json --side left --walldist 0.75 --timeout 180 --note "try to visit artifact and return home" 2>&1 | python subt/std2ros.py

