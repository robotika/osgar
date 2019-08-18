#!/bin/bash

python -m osgar.record ../../config/test-vesc.json &
VESC_PID=$!

python -m subt run ../../config/kloubak2-subt-estop-lora.json --side right --speed 0.3 --timeout 20

kill ${VESC_PID}
wait

