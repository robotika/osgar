#!/usr/bin/env bash

termtitle() { printf "\033]0;$*\007"; }

trap "kill %1" EXIT

ROBOT="${ROBOT:-X0F200L}"
WORLD="${WORLD:-urban_circuit_practice_01}"
CIRCUIT="${CIRCUIT:-urban}"

( while true; do termtitle "bridge $ROBOT $WORLD"; sleep 5; done ) &

cd $(git rev-parse --show-toplevel)

./subt/docker/run.bash osrf/subt-virtual-testbed:cloudsim_bridge_latest \
  circuit:=$CIRCUIT \
  worldName:=$WORLD \
  robotName1:=$ROBOT \
  robotConfig1:=SSCI_X4_SENSOR_CONFIG_2

