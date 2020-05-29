#!/bin/bash

echo "Waiting for robot name"
while [ -z "$ROBOT_NAME" ]; do
    ROBOT_NAME=$(rosparam get /robot_names 2>/dev/null)
    sleep 0.5
done
echo "Robot name is '$ROBOT_NAME'"

/opt/ros/melodic/lib/teleop_twist_keyboard/teleop_twist_keyboard.py cmd_vel:=/${ROBOT_NAME}/cmd_vel_osgar __name:=keyboard
