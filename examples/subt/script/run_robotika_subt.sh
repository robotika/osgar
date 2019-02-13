#!/bin/bash

# Cleanup
rm -rf ~/.gazebo/log/*
rm -f ~/md/osgar/examples/subt/call_base.txt
rm -f ~/md/osgar/examples/subt/call_base_x2l.txt
rm -f ~/md/osgar/examples/subt/call_base_x2r.txt
source ~/subt_ws/install/setup.sh

# Start gazebo.
export DISPLAY=:0  # Even when connected through ssh, Gazebo needs a local window.
roslaunch subt_gazebo competition.launch scenario:=tunnel_qual extra_gazebo_args:="-r" &
GAZEBO_PID=$!
sleep 10

# Set up robots.
X2_SENSOR_CONFIG_4=1 DISPLAY=:0 roslaunch subt_example x2lr_team.launch &
ROBOTS_SIM_PID=$!
sleep 15

# Control the robots.
export PYTHONPATH=${PYTHONPATH}:~/md/osgar
cd ~/md/osgar/examples/subt
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

# Take robot simulation down.
kill ${ROBOTS_SIM_PID}
kill ${GAZEBO_PID}

# Convert the log.
#time gz log -f ~/.gazebo/log/*/gzserver/state.log --filter *.pose/*.pose -z 60 -o ~/.gazebo/log/subt_tunnel_qual_sim_state.log

