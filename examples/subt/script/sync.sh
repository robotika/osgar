#!/bin/bash

cp osgar/examples/subt/src/subt_example_node.cc src/subt/subt_example/src/subt_example_node.cc
cp osgar/examples/subt/src/CMakeLists.txt src/subt/subt_example/CMakeLists.txt
cp osgar/examples/subt/launch/x2lr_team.launch src/subt/subt_example/launch/x2lr_team.launch
cp osgar/examples/subt/config/robot_config.yaml src/subt/subt_example/config/robot_config.yaml

catkin_make install

