#!/bin/bash

# NASA qualification round number
ROUND=3

# after how long should the simulation give up (in seconds); note the round is allowed to run for up to 45 minutes
TIMEOUT=1800

# desired score
SCORE="13"

RANGE_START=0
RANGE_END=49

SRCP2_WAIT=5

while getopts "r:t:s:f:l:hw:" opt; do
    case ${opt} in
        h)
            echo "Usage:"
            echo " -r <qualification round>"
            echo " -t <timeout in secs>"
            echo " -s <desired score>"
            echo " -f <first seed range>"
            echo " -l <last seed range>"
            echo " -w <SRCP2 simulator startup wait>"
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
        w)
            SRCP2_WAIT=$OPTARG
            ;;
    esac
done

now=$(date +"%m%d%Y_%H%M")
log_dir="bulklogs_${now}"
mkdir ${log_dir}

for i in $(seq $RANGE_START $RANGE_END)
do

    echo "==========" `date` "====== seed:" $i
    $HOME/space-challenge/srcp2-competitors/docker/scripts/launch/roslaunch_docker -q -n --run-round $ROUND -s $i >& /dev/null

    # wait for SRCP2 simulator to start
    sleep ${SRCP2_WAIT}

    osgar_image_id=`docker image ls | grep rover | grep latest | awk '{print $3}'`
    srcp2_image_id=`docker image ls | grep comp | head -1 | awk '{print $3}'`
    echo "Running OSGAR image ${osgar_image_id}" >> ${log_dir}/osgar-tty-${now}-seed_${i}.txt
    echo "Running SRCP2 image ${srcp2_image_id}" >> ${log_dir}/osgar-tty-${now}-seed_${i}.txt

    timeout $TIMEOUT sh -c "( docker run --rm --network=host -t --name nasa-rover rover:latest /osgar/moon/docker/rover/run_solution.bash -r $ROUND &) | tee -a ${log_dir}/osgar-tty-${now}-seed_${i}.txt | grep -m 1 \"Score: $SCORE\""

    for f in $(docker exec -it nasa-rover /bin/bash -c "ls /*.log"); do
        f=`echo $f | sed 's/\r//g'`
        readarray -d . -t strarr <<< "$f"
        docker cp nasa-rover:$f ${log_dir}/${strarr[0]}-seed_${i}.${strarr[1]};
    done

    docker kill nasa-rover >& /dev/null
    docker kill srcp2-simulation >& /dev/null

done
