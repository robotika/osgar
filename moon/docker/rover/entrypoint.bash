#!/bin/bash
set -e

# these ROS workspaces are already sourced into the openvslam environment, no need to source separately
#source /opt/ros/melodic/setup.bash
#source /srcp2-competitors/ros_workspace/install/setup.bash

source "/openvslam/ros/1/devel/setup.bash"

exec "$@"
