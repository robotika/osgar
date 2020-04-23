#!/bin/bash

source ~/subt_solution/devel/setup.sh
# We need to look for NVidia libraries, but they are not around yet when
# building the image, so `ldconfig` in Dockerfile is not a working solution.
# https://gitlab.com/nvidia/container-images/cuda/-/issues/34
sudo ldconfig

exec "$@"
