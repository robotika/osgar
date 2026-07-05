#!/bin/bash

set -e  # Exit on any error.

PKGS_SRC_PATH=$(dirname "$0")/../ros
CATKIN_DEVEL_PATH=ros.devel.tmp
INSTALL_PATH=ros/subt
echo "Building Osgar SubT ROS modules ..."
catkin_make --source "${PKGS_SRC_PATH}" -DCATKIN_DEVEL_PREFIX="${CATKIN_DEVEL_PATH}" -DCMAKE_INSTALL_PREFIX="${INSTALL_PATH}"
echo "Installing Osgar SubT ROS modules ..."
catkin_make install --source "${PKGS_SRC_PATH}" -DCATKIN_DEVEL_PREFIX="${CATKIN_DEVEL_PATH}" -DCMAKE_INSTALL_PREFIX="${INSTALL_PATH}"
echo "Osgar SubT ROS modules are installed in \"${INSTALL_PATH}\"."
