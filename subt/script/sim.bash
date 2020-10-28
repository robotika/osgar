#!/usr/bin/env bash

termtitle() { printf "\033]0;$*\007"; }

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

HEADLESS="${HEADLESS:-false}"
CONFIG="${CONFIG:-ROBOTIKA_X2_SENSOR_CONFIG_1}"

cd "$(git rev-parse --show-toplevel)" || exit

LOG_DIR="$(pwd)/ign-logs/$(date +%Y-%m-%dT%H.%M.%S)"

echo $CIRCUIT
echo $LOG_DIR
echo $ROBOT
echo $WORLD
mkdir -p $LOG_DIR

trap 'echo; echo $CIRCUIT; echo $LOG_DIR; echo $ROBOT; echo $WORLD; echo;' EXIT

DOCKER_OPTS="--volume ${LOG_DIR}:/tmp/ign/logs --name sim"
export DOCKER_OPTS

termtitle "sim $ROBOT $WORLD"

./subt/docker/run.bash osrf/subt-virtual-testbed:cloudsim_sim_latest \
  cloudsim_sim.ign \
  headless:=$HEADLESS \
  seed:=1 \
  circuit:=$CIRCUIT \
  worldName:=$WORLD \
  robotName1:=$ROBOT \
  robotConfig1:=$CONFIG
