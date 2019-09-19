#!/usr/bin/env bash

. /opt/ros/melodic/setup.bash
. ~/subt_solution/install/setup.sh

# Wait for the bridge
sleep 30

# Run your solution.
roslaunch subt_seed x1.launch
