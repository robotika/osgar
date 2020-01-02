#!/usr/bin/env bash

. /opt/ros/melodic/setup.bash
. ~/subt_solution/install/setup.sh

echo "Start robot solution"
export OSGAR_LOGS=`pwd`
cd osgar
python3 -m subt run subt/zmq-subt-x2.json --side auto --walldist 1.5 --timeout 100 --speed 1.0 --note "try to visit artifact and return home" &
ROBOT_PID=$!
cd ..

# Run your solution and wait for ROS master
# http://wiki.ros.org/roslaunch/Commandline%20Tools#line-45
roslaunch subt_seed x1.launch --wait &
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

rosservice call '/subt/finish' true

# Take robot simulation down.
kill ${ROS_PID}
wait

