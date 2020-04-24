FROM osrf/subt-virtual-testbed:subt_solution_latest

SHELL ["/bin/bash", "-c"]

RUN /usr/bin/python3 -m pip install --user --no-cache-dir \
  "msgpack==1.0.0" \
  "numpy==1.18.2" \
  "opencv-python==4.2.0.32" \
  "pyzmq==19.0.0"

RUN cd /home/developer/ && head -n -2 .bashrc > .bashrc.new && mv .bashrc.new .bashrc

COPY --chown=developer:developer . ./src/osgar/

RUN source /opt/ros/melodic/setup.bash && ./src/osgar/subt/script/sync.sh

RUN sudo apt-get update && sudo apt install -y ros-melodic-image-geometry \
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

ENTRYPOINT ["./src/osgar/subt/docker/robotika/entrypoint.bash"]

CMD ["./src/osgar/subt/docker/robotika/run_solution.bash"]
