#!/bin/bash

echo "running wrapped"

xvfb-run -a -s '-screen 0 1x1x24' vglrun -d /dev/dri/card1 ./run_sim.bash $@
