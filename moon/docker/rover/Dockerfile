FROM ros:melodic

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y python-pip python3-pip openssh-client vim less
RUN python -m pip install pyzmq

RUN /usr/bin/python3 -m pip install --user --no-cache-dir msgpack "opencv-python>3.4.2,<4" pyzmq shapely scipy

COPY ./srcp2-competitors ./srcp2-competitors
COPY ./osgar ./osgar

RUN echo "source /srcp2-competitors/ros_workspace/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["./osgar/moon/docker/rover/entrypoint.bash"]

CMD ["/bin/bash"]
