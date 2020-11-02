#!/usr/bin/env bash

# adjust so that local logs look similar to cloudsim logs
export ROSCONSOLE_FORMAT='${time} ${severity} ${node} ${logger}: ${message}'

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

echo "Waiting for robot description"
while [ -z "$ROBOT_DESCRIPTION" ]; do
    ROBOT_DESCRIPTION=$(rosparam get /$ROBOT_NAME/robot_description)
    sleep 0.5
done
echo "Robot description is '$ROBOT_DESCRIPTION'"

grep -q ssci_x4_sensor_config <<< $ROBOT_DESCRIPTION && IS_X4=true || IS_X4=false
grep -q TeamBase <<< $ROBOT_DESCRIPTION && IS_TEAMBASE=true || IS_TEAMBASE=false
grep -q robotika_freyja_sensor_config <<< $ROBOT_DESCRIPTION && IS_FREYJA=true || IS_FREYJA=false
grep -q explorer_r2_sensor_config <<< $ROBOT_DESCRIPTION && IS_EXPLORER_R2=true || IS_EXPLORER_R2=false
grep -q robotika_kloubak_sensor_config <<< $ROBOT_DESCRIPTION && IS_K2=true || IS_K2=false

# Defaults
WALLDIST=0.8
SPEED=1.0

if $IS_X4
then
    echo "Robot is X4 drone"    
    LAUNCH_FILE="robot x4.launch"
    CONFIG_FILES=("zmq-subt-x4.json")
    SPEED=1.5
    WALLDIST=1.2
elif $IS_TEAMBASE
then
    echo "Robot is TEAMBASE"
    LAUNCH_FILE="proxy teambase.launch"
    CONFIG_FILES=("zmq-subt-teambase.json")
elif $IS_FREYJA
then
    echo "Robot is Freyja"
    LAUNCH_FILE="robot freyja.launch"
    CONFIG_FILES=("zmq-subt-x2.json" "subt-freyja.json")
    WALLDIST=1.0
    SPEED=1.5
elif $IS_EXPLORER_R2
then
    echo "Robot is Explorer R2"
    LAUNCH_FILE="robot r2.launch"
    CONFIG_FILES=("config/r2.json")
    WALLDIST=1.5
    SPEED=1.5
elif $IS_K2
then
    echo "Robot is K2"
    LAUNCH_FILE="robot k2-virt.launch"
    CONFIG_FILES=("zmq-subt-x2.json" "subt-k2-virt.json")
    WALLDIST=1.0
    SPEED=1.5
else
    echo "Robot is default X2 wheeled robot"
    LAUNCH_FILE="proxy sim.launch"
    CONFIG_FILES=("zmq-subt-x2.json")
fi

echo "Starting ros<->osgar proxy"
# Wait for ROS master, then start ros nodes
# http://wiki.ros.org/roslaunch/Commandline%20Tools#line-45 
roslaunch $LAUNCH_FILE --wait robot_name:=$ROBOT_NAME &

/osgar-ws/src/osgar/subt/cloudsim2osgar.py $ROBOT_NAME &

LOG_FILE=/osgar-ws/logs/$(basename ${CONFIG_FILES[-1]} .json)-$(date +%Y-%m-%dT%H.%M.%S).log
CONFIG_FILE_PATHS=()
for CONFIG_FILE in ${CONFIG_FILES[@]};
do
  CONFIG_FILE_PATHS+=(/osgar-ws/src/osgar/subt/$CONFIG_FILE)
done
PYTHON=/osgar-ws/env/bin/python3

rosrun proxy sendlog.py $LOG_FILE &

echo "Starting osgar"
if $IS_TEAMBASE
then
    $PYTHON -m subt.teambase ${CONFIG_FILE_PATHS[@]} --robot-name $ROBOT_NAME --log $LOG_FILE --note "run teambase"
else
    $PYTHON -m subt run ${CONFIG_FILE_PATHS[@]} --log $LOG_FILE --side auto --walldist $WALLDIST --speed $SPEED --note "run_solution.bash"
fi



echo "Sleep and finish"
sleep 30

# DO NOT CALL /subt/finish for group of robots!
#
if $IS_TEAMBASE
then
    echo "TEAMBASE is terminating all robots"
    rosservice call '/subt/finish' true
    # it terminates the run for everybody
    # https://bitbucket.org/osrf/subt/issues/336/handling-subt-finish-for-multiple-robots
fi

