FROM osrf/subt-virtual-testbed:cloudsim_sim_latest

RUN cd /tmp && \
    curl -sSO https://s3.amazonaws.com/virtualgl-pr/dev/linux/virtualgl_2.6.80_amd64.deb && \
    sudo apt-get install ./virtualgl_*

#ENTRYPOINT ["/usr/bin/env"]

COPY subt/docker/cloudsim/run_wrapped.bash .

ENTRYPOINT ["./run_wrapped.bash"]
