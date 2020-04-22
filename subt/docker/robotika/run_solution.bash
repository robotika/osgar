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

echo "Waiting for robot name"
while [ -z "$ROBOT_NAME" ]; do
    ROBOT_NAME=$(rosparam get /robot_names)
    sleep 0.5
done
echo "Robot name is '$ROBOT_NAME'"

echo "Start robot solution"
export OSGAR_LOGS=/osgar-ws/logs
/osgar-ws/env/bin/python3 -m subt run /osgar-ws/src/subt/zmq-subt-x2.json --side auto --walldist 0.8 --timeout 100 --speed 1.0 --note "run_solution.bash" &
ROBOT_PID=$!

# Run your solution and wait for ROS master
# http://wiki.ros.org/roslaunch/Commandline%20Tools#line-45
roslaunch proxy sim.launch --wait robot_name:=$ROBOT_NAME &
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

