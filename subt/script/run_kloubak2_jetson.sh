#!/bin/bash

function trap_ctrlc_front ()
{
	  ssh -t k2jetson_front "pkill -15 -f 'osgar.record'"
	    exit 2 
    }

(python -m subt --use-old-record run config/kloubak2-subt-estop-lora-jetson.json --side left --speed 0.5 --timeout 600; python -m osgar.record config/test-lora.json) &

trap "trap_ctrlc_front" 2
ssh -t k2jetson_front "python3 -m osgar.record ~/git/osgar/config/jetson-node-k2-front.json" &
ssh -t k2jetson_rear "python3 -m osgar.record ~/git/osgar/config/jetson-node-k2-rear.json"
wait
