FROM osrf/subt-virtual-testbed:subt_solution_latest

RUN /usr/bin/python3 -m pip install --user --no-cache-dir msgpack pyserial "opencv-python>3.4.2,<4" pyzmq
RUN git clone https://github.com/robotika/osgar.git osgar \
  && cd osgar \
  && git checkout feature/virtual-urban

COPY run_solution.bash ./

COPY ros_proxy_node.cc ./src/subt_seed/src/subt_seed_node.cc
COPY CMakeLists.txt ./src/subt_seed/CMakeLists.txt
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && catkin_make install'

ENTRYPOINT ["./run_solution.bash"]

