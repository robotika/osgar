#!/usr/bin/env bash

# adjust so that local logs look similar to cloudsim logs
export ROSCONSOLE_FORMAT='${time} ${severity} ${node} ${logger}: ${message}'

# when signal received just exit
trap "echo 'got signal... exiting'; exit;" HUP INT TERM

# when exiting, send SIGINT to all processes in current process group
# and wait for them to finish
trap "echo 'exiting... killing children...';  kill -s SIGINT 0; wait" EXIT

rosrun proxy wait.py

ROBOT_NAME=$(rosparam get /robot_names)
echo "Robot name is '$ROBOT_NAME'"

ROBOT_CONFIG=$(rosparam get /robot_config)
echo "Robot config is '$ROBOT_CONFIG'"

# Defaults
WALLDIST=0.8
SPEED=1.0

case $ROBOT_CONFIG in
  "SSCI_X4_SENSOR_CONFIG_2"):
    echo "Robot is X4 drone"
    LAUNCH_FILE="robot x4.launch"
    CONFIG_FILES=("zmq-subt-x4.json")
    SPEED=1.5
    WALLDIST=1.2
    ;;
  "TEAMBASE"):
    echo "Robot is TEAMBASE"
    LAUNCH_FILE="proxy teambase.launch"
    CONFIG_FILES=("zmq-subt-teambase.json")
    ;;
  "ROBOTIKA_FREYJA_SENSOR_CONFIG"*):
    echo "Robot is Freyja"
    LAUNCH_FILE="robot freyja.launch"
    CONFIG_FILES=("zmq-subt-x2.json" "subt-freyja.json")
    WALLDIST=1.0
    SPEED=1.5
    ;;
  "EXPLORER_R2_SENSOR_CONFIG"*):
    echo "Robot is Explorer R2"
    LAUNCH_FILE="robot r2.launch"
    CONFIG_FILES=("config/r2.json")
    WALLDIST=1.5
    SPEED=1.5
    ;;
  "ROBOTIKA_KLOUBAK_SENSOR_CONFIG"*):
    echo "Robot is K2"
    LAUNCH_FILE="robot k2-virt.launch"
    CONFIG_FILES=("zmq-subt-x2.json" "subt-k2-virt.json")
    WALLDIST=1.0
    SPEED=1.5
    ;;
  "ROBOTIKA_X2_SENSOR_CONFIG_1"):
    echo "Robot is default X2 wheeled robot"
    LAUNCH_FILE="proxy sim.launch"
    CONFIG_FILES=("zmq-subt-x2.json")
    ;;
  *):
    echo "unknown robot config";
    exit 1
    ;;
esac

echo "Starting ros<->osgar proxy"
# Wait for ROS master, then start ros nodes
# http://wiki.ros.org/roslaunch/Commandline%20Tools#line-45 
roslaunch $LAUNCH_FILE --wait robot_name:=$ROBOT_NAME &

/osgar-ws/src/osgar/subt/cloudsim2osgar.py $ROBOT_NAME $ROBOT_CONFIG &

if [ -f "$1" ];
then
  LOG_FILE="$1"
else
  LOG_FILE=/osgar-ws/logs/$(basename ${CONFIG_FILES[-1]} .json)-$(date +%Y-%m-%dT%H.%M.%S).log
fi

CONFIG_FILE_PATHS=()
for CONFIG_FILE in ${CONFIG_FILES[@]};
do
  CONFIG_FILE_PATHS+=(/osgar-ws/src/osgar/subt/$CONFIG_FILE)
done
PYTHON=/osgar-ws/env/bin/python3

rosrun proxy sendlog.py $LOG_FILE &

echo "Starting osgar" ${CONFIG_FILE_PATHS[@]}
if [ $ROBOT_CONFIG == "TEAMBASE" ]
then
    $PYTHON -m subt.teambase ${CONFIG_FILE_PATHS[@]} --robot-name $ROBOT_NAME --log $LOG_FILE --note "run teambase" &
else
    $PYTHON -m subt run ${CONFIG_FILE_PATHS[@]} --log $LOG_FILE --side auto --walldist $WALLDIST --speed $SPEED --note "run_solution.bash" &
fi

# https://linuxconfig.org/how-to-propagate-a-signal-to-child-processes-from-a-bash-script
echo "Expecting signals while waiting for osgar to finish..."
wait $!


echo "Sleep and finish"
sleep 30

# DO NOT CALL /subt/finish for group of robots!
#
if [ $ROBOT_CONFIG == "TEAMBASE" ]
then
    echo "TEAMBASE is terminating all robots"
    rosservice call '/subt/finish' true
    # it terminates the run for everybody
    # https://bitbucket.org/osrf/subt/issues/336/handling-subt-finish-for-multiple-robots
fi

