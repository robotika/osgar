#!/usr/bin/env bash

# disable crazy threading behavior inside openblas
# https://github.com/xianyi/OpenBLAS#setting-the-number-of-threads-using-environment-variables
export OMP_NUM_THREADS=1

# adjust so that local logs look similar to cloudsim logs
export ROSCONSOLE_FORMAT='${time} ${severity} ${node} ${logger}: ${message}'

# get directory where this bash script lives
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# enable ROS DEBUG output to see if messages are being dropped
export ROSCONSOLE_CONFIG_FILE="${DIR}/rosconsole.config"

# when signal received just exit
trap "exit;" HUP INT TERM

# when exiting, send SIGINT to all processes in current process group
# and wait for them to finish
trap "kill -s SIGINT 0; wait" EXIT

echo "Waiting for robot name"
while [ -z "$ROBOT_NAME" ]; do
    ROBOT_NAME=$(rosparam get /robot_names)
    sleep 0.5
done
echo "Robot name is '$ROBOT_NAME'"

echo "Starting ros<->osgar proxy"
# Wait for ROS master, then start ros nodes
# http://wiki.ros.org/roslaunch/Commandline%20Tools#line-45
roslaunch robotika robot.launch --wait robot_name:=$ROBOT_NAME &

echo "Starting osgar"
export OSGAR_LOGS=/osgar-ws/logs
/osgar-ws/env/bin/python3 -m subt run /osgar-ws/src/osgar/subt/zmq-subt-x4.json --side auto --walldist 0.8 --timeout 100 --speed 1.0 --note "run_solution.bash"

echo "Sleep and finish"
sleep 30

# DO NOT CALL /subt/finish for group of robots!
# it terminates the run for everybody
# https://bitbucket.org/osrf/subt/issues/336/handling-subt-finish-for-multiple-robots
# rosservice call '/subt/finish' true
