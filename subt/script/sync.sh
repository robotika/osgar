#!/bin/bash

cp osgar/subt/src/ros_proxy_node.cc src/subt/subt_example/src/subt_example_node.cc
cp osgar/subt/src/CMakeLists.txt src/subt/subt_example/CMakeLists.txt

catkin_make install

