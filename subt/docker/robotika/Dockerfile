FROM robotika/subt-base:2020-09-22

RUN curl -O# http://osgar.robotika.cz/subt/virtual/model/mdnet3.64.64.13.4.relu.pth
RUN curl -O#  http://osgar.robotika.cz/subt/virtual/model/ssd_inception_v2_201009/frozen_inference_graph.pb
RUN curl -O#  http://osgar.robotika.cz/subt/virtual/model/ssd_inception_v2_201009/cv_graph.pbtxt

RUN sudo apt-get update && sudo apt install -y ros-melodic-teleop-twist-keyboard

ENTRYPOINT ["./src/osgar/subt/docker/robotika/entrypoint.bash"]

CMD ["./src/osgar/subt/docker/robotika/run_solution.bash"]

# support `docker exec -it <container> bash
RUN echo "source /osgar-ws/devel/setup.sh" >> ~/.bashrc

# copy whole build context
COPY . ./src/osgar/

RUN /osgar-ws/env/bin/pip install -e src/osgar/

RUN source /opt/subt/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release

ENV ROSCONSOLE_CONFIG_FILE=/osgar-ws/src/osgar/subt/docker/robotika/rosconsole.config
ENV ROS_PYTHON_LOG_CONFIG_FILE=/osgar-ws/src/osgar/subt/docker/robotika/python_logging.conf

RUN ln -s /osgar-ws/src/osgar/subt/docker/robotika/Makefile .
RUN ln -s /osgar-ws/src/osgar/subt/docker/robotika/run_solution.bash .
