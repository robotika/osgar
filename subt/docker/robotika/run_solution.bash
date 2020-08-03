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
ROBOT_DESCRIPTION=$(rosparam get /$ROBOT_NAME/robot_description)
grep -q ssci_x4_sensor_config <<< $ROBOT_DESCRIPTION && IS_X4=true || IS_X4=false
grep -q TeamBase <<< $ROBOT_DESCRIPTION && IS_TEAMBASE=true || IS_TEAMBASE=false
if $IS_X4
then
    echo "Robot is X4 drone"    
    LAUNCH_FILE="robot drone_keyboard.launch"
    CONFIG_FILE="zmq-subt-x4.json"
elif $IS_TEAMBASE
then
    echo "Robot is TEAMBASE"
    LAUNCH_FILE="proxy sim.launch"
    CONFIG_FILE="zmq-teambase.json"
else
    echo "Robot is X2 wheeled robot"
    LAUNCH_FILE="proxy sim.launch"
    CONFIG_FILE="zmq-subt-x2.json"
fi

echo "Starting ros<->osgar proxy"
# Wait for ROS master, then start ros nodes
# http://wiki.ros.org/roslaunch/Commandline%20Tools#line-45 
roslaunch $LAUNCH_FILE --wait robot_name:=$ROBOT_NAME &

/osgar-ws/src/osgar/subt/cloudsim2osgar.py $ROBOT_NAME &

LOG_FILE=/osgar-ws/logs/$(basename $CONFIG_FILE .json)-$(date +%Y-%m-%dT%H.%M.%S).log
CONFIG_FILE=/osgar-ws/src/osgar/subt/$CONFIG_FILE
PYTHON=/osgar-ws/env/bin/python3

rosrun proxy sendlog.py $LOG_FILE &

echo "Starting osgar"
$PYTHON -m subt run $CONFIG_FILE --log $LOG_FILE --side auto --walldist 0.8 --speed 1.0 --note "run_solution.bash"

echo "Sleep and finish"
sleep 30

# DO NOT CALL /subt/finish for group of robots!
# it terminates the run for everybody
# https://bitbucket.org/osrf/subt/issues/336/handling-subt-finish-for-multiple-robots
# rosservice call '/subt/finish' true
