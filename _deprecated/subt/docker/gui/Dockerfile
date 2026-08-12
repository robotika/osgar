FROM osrf/subt-virtual-testbed:cloudsim_sim_latest

RUN mkdir -p /home/developer/.ignition/gazebo/

RUN cd /home/developer/.ignition/fuel/fuel.ignitionrobotics.org/OpenRobotics/models/ && \
    ln -s Robotika_Freyja_Sensor_Config_2 ROBOTIKA_FREYJA_SENSOR_CONFIG_2

COPY ./subt/docker/gui/entrypoint.bash .

ENTRYPOINT ["./entrypoint.bash"]

ENV IGN_PARTITION=sim

CMD ["ign", "gazebo", "-g"]
