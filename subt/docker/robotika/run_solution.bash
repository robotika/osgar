#!/usr/bin/env bash

. /opt/ros/melodic/setup.bash
. ~/subt_solution/install/setup.sh

echo "Start robot solution"
export OSGAR_LOGS=`pwd`
cd osgar
taskset -ac 0 python3 -m subt run subt/zmq-subt-x2.json --side auto --walldist 0.8 --timeout 100 --speed 1.0 --note "try to visit artifact and return home" &
ROBOT_PID=$!
cd ..

# get directory where this bash script lives
samedir=$(dirname $(readlink -f "${BASH_SOURCE[0]}"))

# enable ROS DEBUG output to see if messages are being dropped
export ROSCONSOLE_CONFIG_FILE="${samedir}/rosconsole.config"

# Run your solution and wait for ROS master
# http://wiki.ros.org/roslaunch/Commandline%20Tools#line-45
taskset -ac 0 nice -n 19 roslaunch subt_seed x1.launch --wait &
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

# DO NOT CALL /subt/finish for group of robots!
# it terminates the run for everybody
# https://bitbucket.org/osrf/subt/issues/336/handling-subt-finish-for-multiple-robots
# rosservice call '/subt/finish' true

# Take robot simulation down.
kill ${ROS_PID}
wait

