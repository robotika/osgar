#!/usr/bin/env bash

. /opt/ros/melodic/setup.bash
. ~/subt_solution/install/setup.sh

echo "Wait for the bridge"
sleep 30

echo "Start robot solution"
cd osgar
python3 -m subt run subt/zmq-subt-x2.json --side auto --walldist 0.5 --timeout 100 --speed 1.0 --note "try to visit artifact and return home" &
ROBOT_PID=$!
cd ..

# Run your solution.
roslaunch subt_seed x1.launch &
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

echo "Dump logfile"
cd osgar
ls -lh | python3 -m osgar.record subt/config/stdout2ros.json --duration 10
sleep 1
cat zmq-subt-x2-*.log | base64 | python3 -m osgar.record subt/config/stdout2ros.json --duration 600
cd ..
echo "Dump completed"


rosservice call '/subt/finish' true

# Take robot simulation down.
kill ${ROS_PID}
wait

echo "10 minutes for logfile download"
sleep 600
