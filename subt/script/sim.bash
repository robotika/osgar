#!/usr/bin/env bash

termtitle() { printf "\033]0;$*\007"; }

ROBOT="${ROBOT:-X0F200L}"
WORLD="${WORLD:-urban_circuit_practice_01}"
HEADLESS="${HEADLESS:-false}"

cd "$(git rev-parse --show-toplevel)" || exit

LOG_DIR="$(pwd)/ign-logs/$(date +%Y-%m-%dT%H.%M.%S)"

echo $LOG_DIR
mkdir -p $LOG_DIR

trap 'echo; echo $LOG_DIR; echo $ROBOT; echo $WORLD; echo;' EXIT

DOCKER_OPTS="--volume ${LOG_DIR}:/tmp/ign/logs"
export DOCKER_OPTS

termtitle "sim $ROBOT $WORLD"

./subt/docker/run.bash osrf/subt-virtual-testbed:cloudsim_sim_latest \
  cloudsim_sim.ign \
  headless:=$HEADLESS \
  seed:=1 \
  circuit:=urban \
  worldName:=$WORLD \
  robotName1:=$ROBOT \
  robotConfig1:=ROBOTIKA_X2_SENSOR_CONFIG_1
