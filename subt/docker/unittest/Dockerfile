FROM osrf/subt-virtual-testbed:subt_solution_latest

COPY subt/docker/unittest/run_solution.bash ./

COPY subt/docker/unittest/subt_seed_node.cc ./src/subt_seed/src/subt_seed_node.cc
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && catkin_make install'

ENTRYPOINT ["./run_solution.bash"]

