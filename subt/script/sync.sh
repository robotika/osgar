#!/bin/bash

cp osgar/subt/src/subt_example_node.cc src/subt/subt_example/src/subt_example_node.cc
cp osgar/subt/launch/example_robot.launch src/subt/subt_example/launch/example_robot.launch

catkin_make install

