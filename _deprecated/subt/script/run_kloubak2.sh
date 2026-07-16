#!/bin/bash

python -m subt run config/kloubak2-subt-estop-lora.json --side left --speed 0.7 --timeout 1200
python -m osgar.record config/test-lora.json
