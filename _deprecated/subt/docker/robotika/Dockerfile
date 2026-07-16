FROM robotika/subt-base:2021-05-24

# ADD always downloads the file to calculate checksum, RUN checksum only the cmdline
RUN curl -O# http://osgar.robotika.cz/subt/virtual/model/mdnet6.128.128.13.4.elu.pth 2>&1
RUN curl -O# http://osgar.robotika.cz/subt/virtual/model/ssd_inception_v2_210814/frozen_inference_graph.pb 2>&1
RUN curl -O# http://osgar.robotika.cz/subt/virtual/model/ssd_inception_v2_210814/cv_graph.pbtxt 2>&1

# example how to install an ubuntu package
# RUN sudo apt-get update && sudo apt install -y ros-melodic-teleop-twist-keyboard

ENTRYPOINT ["./src/osgar/subt/docker/robotika/entrypoint.bash"]

CMD ["./src/osgar/subt/docker/robotika/run_solution.bash"]

# support `docker exec -it <container> bash
RUN echo "source /osgar-ws/devel/setup.sh" >> ~/.bashrc

# copy whole build context
COPY . ./src/osgar/

RUN /osgar-ws/env/bin/pip install --no-cache-dir -e src/osgar/

RUN source /opt/subt/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release

ENV ROSCONSOLE_CONFIG_FILE=/osgar-ws/src/osgar/subt/docker/robotika/rosconsole.config
ENV ROS_PYTHON_LOG_CONFIG_FILE=/osgar-ws/src/osgar/subt/docker/robotika/python_logging.conf

# adjust so that local logs look similar to cloudsim logs
ENV ROSCONSOLE_FORMAT='${time} ${severity} ${node} ${logger}: ${message}'

RUN ln -s /osgar-ws/src/osgar/subt/docker/robotika/Makefile .
RUN ln -s /osgar-ws/src/osgar/subt/docker/robotika/run_solution.bash .
