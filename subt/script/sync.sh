#!/bin/bash

cp osgar/subt/src/ros_proxy_node.cc src/subt_seed/src/subt_seed_node.cc
cp osgar/subt/src/CMakeLists.txt src/subt_seed/CMakeLists.txt

catkin_make install

