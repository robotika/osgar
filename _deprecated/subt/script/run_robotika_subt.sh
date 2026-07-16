#!/bin/bash

# Cleanup
rm -f osgar/examples/subt/call_base.txt
rm -f osgar/examples/subt/call_base_x2l.txt
rm -f osgar/examples/subt/call_base_x2r.txt
source ~/subt_ws/install/setup.sh

echo Starting gazebo....
export DISPLAY=:0  # Even when connected through ssh, Gazebo needs a local window.
roslaunch subt_gazebo competition.launch scenario:=tunnel_qual extra_gazebo_args:="-r --record_period 0.0167 --record_filter *.pose/*.pose" &
GAZEBO_PID=$!
# wait for gazebo to fully load
while true; do
	rostopic type /subt/score >/dev/null 2>/dev/null && break
	sleep 1
done

sleep 1

# Set up robots.
X2_SENSOR_CONFIG_4=1 DISPLAY=:0 roslaunch subt_example x2lr_team.launch &
ROBOTS_SIM_PID=$!
while true; do
	x2r=$(rostopic list | grep X2R | wc -l)
	x2l=$(rostopic list | grep X2L | wc -l)
	if [ $x2l -ne 0 -a $x2l -eq $x2r ]; then
	        break
        else
		echo "still waiting for robots to be loaded..."
		sleep 1
	fi
done

sleep 1

# Control the robots.
export OSGAR_LOGS=$(pwd)/osgar/examples/subt/logs
mkdir $OSGAR_LOGS
export PYTHONPATH=${PYTHONPATH}:$(pwd)/osgar
cd osgar/examples/subt
python3 subt.py run subt-x2-left.json &
ROBOT_LEFT_PID=$!
python3 subt.py run subt-x2-right.json &
ROBOT_RIGHT_PID=$!

# Turn everything off in case of CTRL+C and friends.
function shutdown {
       kill ${ROBOT_RIGHT_PID}
       kill ${ROBOT_LEFT_PID}
       kill ${ROBOTS_SIM_PID}
       kill ${GAZEBO_PID}
       wait
       exit
}
trap shutdown SIGHUP SIGINT SIGTERM

# Turn lights on robots on.
rosservice call '/X2L/left_headlight/enable' true
rosservice call '/X2L/right_headlight/enable' true
rosservice call '/X2L/center_left_headlight/enable' true
rosservice call '/X2L/center_right_headlight/enable' true

rosservice call '/X2R/left_headlight/enable' true
rosservice call '/X2R/right_headlight/enable' true
rosservice call '/X2R/center_left_headlight/enable' true
rosservice call '/X2R/center_right_headlight/enable' true

# Wait for the controllers to finish.
wait ${ROBOT_LEFT_PID}
wait ${ROBOT_RIGHT_PID}

rosservice call '/subt/finish' true

# Take robot simulation down.
kill ${ROBOTS_SIM_PID}
kill ${GAZEBO_PID}
wait

