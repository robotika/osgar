#!/usr/bin/env bash

ROBOT="${ROBOT:-X0F200L}"
WORLD="${WORLD:-urban_circuit_practice_01}"

./run.bash osrf/subt-virtual-testbed:cloudsim_bridge_latest \
  circuit:=urban \
  worldName:=$WORLD \
  robotName1:=$ROBOT \
  robotConfig1:=ROBOTIKA_X2_SENSOR_CONFIG_1

