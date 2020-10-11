#!/usr/bin/env bash

termtitle() { printf "\033]0;$*\007"; }

trap "kill %1" EXIT

ROBOT="${ROBOT:-X200L}"
WORLD="${WORLD:-urban_circuit_practice_01}"

case $WORLD in
 *"urban"*):
    CIRCUIT="urban" ;;
 *"cave"*):
    CIRCUIT="cave"; ;;
 *"tunnel"*):
    CIRCUIT="tunnel" ;;
 *):
    echo "circuit not detected";
    exit 1;;
esac

CONFIG="${CONFIG:-ROBOTIKA_X2_SENSOR_CONFIG_1}"

( while true; do termtitle "bridge $ROBOT $WORLD"; sleep 5; done ) &

cd $(git rev-parse --show-toplevel)

DOCKER_OPTS="--name bridge"
export DOCKER_OPTS

echo circuit: $CIRCUIT; echo robot: $ROBOT; echo world: $WORLD;

./subt/docker/run.bash osrf/subt-virtual-testbed:cloudsim_bridge_latest \
  circuit:=$CIRCUIT \
  worldName:=$WORLD \
  robotName1:=$ROBOT \
  robotConfig1:=$CONFIG

