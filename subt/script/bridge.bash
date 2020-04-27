#!/usr/bin/env bash

termtitle() { printf "\033]0;$*\007"; }

trap "kill %1" EXIT

ROBOT="${ROBOT:-X0F200L}"
WORLD="${WORLD:-urban_circuit_practice_01}"

( while true; do termtitle "bridge $ROBOT $WORLD"; sleep 5; done ) &

cd $(git rev-parse --show-toplevel)

./subt/docker/run.bash osrf/subt-virtual-testbed:cloudsim_bridge_latest \
  circuit:=urban \
  worldName:=$WORLD \
  robotName1:=$ROBOT \
  robotConfig1:=ROBOTIKA_X2_SENSOR_CONFIG_1

