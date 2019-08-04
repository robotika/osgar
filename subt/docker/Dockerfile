FROM nkoenig/subt-virtual-testbed

RUN /usr/bin/python3 -m pip install --user --no-cache-dir msgpack pyserial "opencv-python>3.4.2,<4"
RUN git clone https://github.com/robotika/osgar.git osgar \
  && cd osgar \
  && git checkout feature/virtual-ignition

