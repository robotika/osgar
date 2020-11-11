FROM osrf/subt-virtual-testbed:cloudsim_sim_latest

RUN mkdir -p /home/developer/.ignition/gazebo/

COPY ./subt/docker/gui/entrypoint.bash .

ENTRYPOINT ["./entrypoint.bash"]

ENV IGN_PARTITION=sim

CMD ["ign", "gazebo", "-g"]
