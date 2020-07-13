#!/bin/bash

# NASA qualification round number
ROUND=3

# after how long should the simulation give up (in seconds); note the round is allowed to run for up to 45 minutes
TIMEOUT=1800

# desired score
SCORE="13"

RANGE_START=0
RANGE_END=49

while getopts "r:t:s:f:l:h" opt; do
    case ${opt} in
        h)
            echo "Usage:"
            echo " -r <qualification round>"
            echo " -t <timeout in secs>"
            echo " -s <desired score>"
            echo " -f <first seed range>"
            echo " -l <last seed range>"
            exit 0
            ;;
        r)
            ROUND=$OPTARG
            ;;
        t)
            TIMEOUT=$OPTARG
            ;;
        s)
            SCORE=$OPTARG
            ;;
        f)
            RANGE_START=$OPTARG
            ;;
        l)
            RANGE_END=$OPTARG
            ;;
    esac
done


for i in $(seq $RANGE_START $RANGE_END)
do

    echo "==========" `date` "====== seed:" $i
    $HOME/space-challenge/srcp2-competitors/docker/scripts/launch/roslaunch_docker -q -n --run-round $ROUND -s $i >& /dev/null

    # wait 10 secs to start
    sleep 5

    timeout $TIMEOUT sh -c "( docker run --rm --network=host -t --name nasa-rover rover:latest /osgar/moon/docker/rover/run_solution.bash -r $ROUND &) | grep -m 1 \"Score: $SCORE\""

    docker kill nasa-rover >& /dev/null
    docker kill srcp2-simulation >& /dev/null

done
