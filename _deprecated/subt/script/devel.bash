#!/usr/bin/env bash

cd "$(git rev-parse --show-toplevel)" || exit

DIR=$(pwd)

DOCKER_OPTS="--volume ${DIR}:/osgar-ws/src/osgar --name devel"
export DOCKER_OPTS

./subt/docker/run.bash robotika:latest bash
