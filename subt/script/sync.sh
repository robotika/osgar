#!/bin/bash

cp osgar/subt/docker/robotika/ros_proxy_node.cc src/subt_seed/src/subt_seed_node.cc

sum1=$(md5sum osgar/subt/docker/robotika/CMakeLists.txt | cut -d' ' -f1)
sum2=$(md5sum src/subt_seed/CMakeLists.txt | cut -d' ' -f1)

if [ "$sum1" != "$sum2" ]; then
    cp osgar/subt/docker/robotika/CMakeLists.txt src/subt_seed/CMakeLists.txt
fi

catkin_make install -DCMAKE_BUILD_TYPE=Release

