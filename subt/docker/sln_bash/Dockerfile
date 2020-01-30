FROM osrf/subt-virtual-testbed:subt_solution_latest

ARG user_id
RUN sudo groupmod -g ${user_id} developer
RUN sudo usermod -u ${user_id} developer

RUN /usr/bin/python3 -m pip install --user --no-cache-dir msgpack "opencv-python>3.4.2,<4" pyzmq
RUN git clone https://github.com/robotika/osgar.git osgar \
  && cd osgar \
  && git checkout feature/virtual-urban

ENTRYPOINT ["/bin/bash"]

