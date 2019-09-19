FROM osrf/subt-virtual-testbed

RUN /usr/bin/python3 -m pip install --user --no-cache-dir msgpack pyserial "opencv-python>3.4.2,<4"
RUN git clone https://github.com/robotika/osgar.git osgar \
  && cd osgar \
  && git checkout feature/virtual-ignition

# Create a directory required by cloudsim (?!)
RUN mkdir -p /tmp/ign

# Copy entry point script, and set the entrypoint
COPY run_solution.bash ./
ENTRYPOINT ["./run_solution.bash"] 

