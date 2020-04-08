#!/usr/bin/env bash

ROBOT="${ROBOT:-X0F100L}"
WORLD="${WORLD:-urban_circuit_practice_01}"

cd $(git rev-parse --show-toplevel)

./subt/docker/run.bash osrf/subt-virtual-testbed:cloudsim_bridge_latest \
  circuit:=urban \
  worldName:=$WORLD \
  robotName1:=$ROBOT \
  robotConfig1:=ROBOTIKA_X2_SENSOR_CONFIG_1

