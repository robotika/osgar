#!/usr/bin/env bash

ROBOT="${ROBOT:-X0F200L}"
WORLD="${WORLD:-urban_circuit_practice_01}"

cd $(git rev-parse --show-toplevel)

./subt/docker/run.bash osrf/subt-virtual-testbed:cloudsim_sim_latest cloudsim_sim.ign \
  circuit:=urban \
  worldName:=$WORLD \
  robotName1:=$ROBOT \
  robotConfig1:=ROBOTIKA_X2_SENSOR_CONFIG_1
