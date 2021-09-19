#!/bin/bash

function trap_ctrlc_front ()
{
	  ssh -t k2jetson_front "pkill -15 -f 'run_jetson_front.bash'"
	    exit 2 
    }

(python -m subt --use-old-record run config/kloubak2-subt-estop-lora-jetson.json --side right --speed 0.5 --timeout 600 --gap-size 0.6 --wall-dist 0.6 --start-paused; python -m osgar.record config/test-lora.json) &

trap "trap_ctrlc_front" 2
ssh -t k2jetson_front "./subt/script/run_jetson_front.bash" &
ssh -t k2jetson_rear "python3 -m osgar.record ~/git/osgar/config/jetson-node-k2-rear.json"
wait
