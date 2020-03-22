FROM ros:melodic

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y python-pip python3-pip openssh-client vim less
RUN python -m pip install pyzmq

RUN /usr/bin/python3 -m pip install --user --no-cache-dir msgpack "opencv-python>3.4.2,<4" pyzmq

COPY . ./srcp2-competitors/
COPY . ./osgar/

RUN source ./srcp2-competitors/ros_workspace/install/setup.bash

ENTRYPOINT ["./osgar/moon/docker/rover/entrypoint.bash"]

CMD ["./osgar/moon/docker/rover/run_solution.bash"]
