FROM osrf/subt-virtual-testbed:subt_solution_latest

SHELL ["/bin/bash", "-c"]

RUN /usr/bin/python3 -m pip install --user --no-cache-dir \
  "msgpack==1.0.0" \
  "numpy==1.18.2" \
  "opencv-python==4.2.0.32" \
  "pyzmq==19.0.0"

RUN cd /home/developer/ && head -n -2 .bashrc > .bashrc.new && mv .bashrc.new .bashrc
RUN sudo apt update

COPY --chown=developer:developer . ./osgar/

RUN source /opt/ros/melodic/setup.bash && ./osgar/subt/script/sync.sh

#COPY subt_seed_node.cc ./src/subt_seed/src/subt_seed_node.cc
#COPY CMakeLists.txt ./src/subt_seed/CMakeLists.txt

ADD ./subt/docker/robotika/ros ./src

RUN sudo apt install -y ros-melodic-image-geometry \
			ros-melodic-pcl-ros \
			ros-melodic-costmap-2d \
			libdxflib-dev \
			ros-melodic-rtabmap-ros \
			ros-melodic-teleop-twist-keyboard \
			ros-melodic-move-base \
			ros-melodic-dwa-local-planner \
			ros-melodic-robot-localization \
			ros-melodic-depthimage-to-laserscan \
			mc

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && catkin_make install'

ENTRYPOINT ["./osgar/subt/docker/robotika/entrypoint.bash"]

CMD ["./osgar/subt/docker/robotika/run_solution.bash"]
