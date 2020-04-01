#!/bin/bash

cp osgar/subt/docker/robotika/ros_proxy_node.cc src/subt_seed/src/subt_seed_node.cc
cp osgar/subt/docker/robotika/CMakeLists.txt src/subt_seed/CMakeLists.txt

catkin_make install -DCMAKE_BUILD_TYPE=Release

