#!/usr/bin/env bash

echo "Unpause simulation"
rosservice call /gazebo/unpause_physics "{}"
echo "wait a moment"
sleep 5

echo "Start robot solution"
export OSGAR_LOGS=`pwd`
cd osgar
python3 -m osgar.record --duration 2700 moon/scout1.json --note "collect some ROS data" &
ROBOT_PID=$!
cd ..

# get directory where this bash script lives
samedir=$(dirname $(readlink -f "${BASH_SOURCE[0]}"))

# enable ROS DEBUG output to see if messages are being dropped
export ROSCONSOLE_CONFIG_FILE="${samedir}/rosconsole.config"

# Run your solution and wait for ROS master
# http://wiki.ros.org/roslaunch/Commandline%20Tools#line-45
## roslaunch subt_seed x1.launch --wait &
python ./osgar/moon/rospy_rover.py &
ROS_PID=$!

# Turn everything off in case of CTRL+C and friends.
function shutdown {
       kill ${ROBOT_PID}
       kill ${ROS_PID}
       wait
       exit
}
trap shutdown SIGHUP SIGINT SIGTERM


# Wait for the controllers to finish.
wait ${ROBOT_PID}

echo "Sleep and finish"
sleep 30

# Take robot simulation down.
kill ${ROS_PID}
wait

