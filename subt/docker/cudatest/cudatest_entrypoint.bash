#!/usr/bin/env bash

echo "source /opt/ros/melodic/setup.sh" >> ~/.bashrc

source /opt/ros/melodic/setup.bash

exec "$@"
