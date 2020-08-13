FROM ros:melodic

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y python-pip python3-pip openssh-client vim less
RUN python -m pip install pyzmq

RUN /usr/bin/python3 -m pip install scikit-build
RUN /usr/bin/python3 -m pip install msgpack pyzmq shapely scipy
RUN /usr/bin/python3 -m pip install --install-option="-j$(nproc)" "opencv-python>3.4.2,<4"

COPY ./srcp2-competitors ./srcp2-competitors
COPY ./osgar ./osgar

RUN echo "source /srcp2-competitors/ros_workspace/install/setup.bash" >> /root/.bashrc

RUN mkdir -p /rover_workspace/src
COPY ./osgar/moon/ros_workspace /rover_workspace
RUN chmod 777 /rover_workspace

RUN rm -rf /rover_workspace/devel
RUN rm -rf /rover_workspace/build
RUN  /bin/bash -c "source /srcp2-competitors/ros_workspace/install/setup.bash; cd /rover_workspace; catkin_make"

ENTRYPOINT ["./osgar/moon/docker/rover/entrypoint.bash"]

CMD ["/bin/bash"]
