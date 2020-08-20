#!/bin/bash

# NASA qualification round number
ROUND=3

# after how long should the simulation give up (in seconds); note the round is allowed to run for up to 45 minutes
TIMEOUT=1800

# desired score
SCORE="13"

RANGE_START=0
RANGE_END=49

SRCP2_WAIT=10

UNSUCCESS_LOGS_ONLY=0

LOG_DIR_PREFIX="."

REPEAT_SEED=1

VSLAM=0

while getopts "r:t:s:f:l:hw:ud:p:v" opt; do
    case ${opt} in
        h)
            echo "Usage:"
            echo " -r <qualification round>"
            echo " -t <timeout in mins>"
            echo " -s <desired score>"
            echo " -f <first seed range>"
            echo " -l <last seed range>"
            echo " -w <SRCP2 simulator startup wait>"
            echo " -u .. only record unsuccessful run logs"
            echo " -d <log dir prefix>"
            echo " -p <times to repeat each seed>"
            echo " -v <launch VSLAM docker>"
            exit 0
            ;;
        r)
            ROUND=$OPTARG
            ;;
        t)
            TIMEOUT=$(expr 60 \* $OPTARG)
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
        u)
            UNSUCCESS_LOGS_ONLY=1
            ;;
        d)
            LOG_DIR_PREFIX=$OPTARG
            ;;
        p)
            REPEAT_SEED=$OPTARG
            ;;
        v)
            VSLAM=1
            ;;
    esac
done

if (docker ps | grep -q rover); then
    echo "OSGAR docker already running, exiting"
    exit 1
fi

if (docker ps | grep -q scheducation); then
    echo "SRCP2 docker already running, exiting"
    exit 1
fi

SUCCESS_STRING="Score: ${SCORE}"
case ${ROUND} in
    1)
        FAIL_STRING="<none>"
        FAIL_SWITCH=""
        ;;
    2)
        FAIL_STRING="<none>"
        FAIL_SWITCH=""
        ;;
    3)
        FAIL_STRING="object location incorrect"
        FAIL_SWITCH="-e \"${FAIL_STRING}\""
        ;;
esac

now=$(date +"%m%d%Y_%H%M")
log_dir="${LOG_DIR_PREFIX}/bulklogs_${now}"
mkdir ${log_dir}

echo "Round: ${ROUND}, Score: ${SCORE}; Fail String: ${FAIL_STRING}; Seed range: ${RANGE_START}-${RANGE_END}; Repeat seeds: ${REPEAT_SEED}; Timeout(min): `expr ${TIMEOUT} / 60`; Unsuccessful logs only: ${UNSUCCESS_LOGS_ONLY}; Log dir: ${log_dir}"

for i in $(seq $RANGE_START $RANGE_END)
do

    for j in $(seq 1 $REPEAT_SEED)
    do

        log_file=${log_dir}/osgar-tty-${now}-seed_${i}_run_${j}.txt

        echo "==========" `date` "====== seed:" $i
        $HOME/space-challenge/srcp2-competitors/docker/scripts/launch/roslaunch_docker -q -n --run-round $ROUND -s ${i} >& /dev/null

        # wait for SRCP2 simulator to start
        sleep ${SRCP2_WAIT}

        if [[ $VSLAM -eq 1 ]]; then
            if [[ $ROUND -eq 2 ]]; then
                docker run --init --network=host --rm --name openvslam1 -t openvslam-rospublish -r excavator_1 >& /dev/null &
                docker run --init --network=host --rm --name openvslam2 -t openvslam-rospublish -r hauler_1 >& /dev/null &
                vslam1_id=`docker image ls | grep openvslam1 | head -1 | awk '{print $3}'`
                vslam2_id=`docker image ls | grep openvslam2 | head -1 | awk '{print $3}'`
                echo "Running OpenVSLAM images ${vslam1_id} and ${vslam1_id}" >> ${log_file}
            else
                docker run --init --network=host --rm --name openvslam -t openvslam-rospublish >& /dev/null &
                vslam_id=`docker image ls | grep openvslam | head -1 | awk '{print $3}'`
                echo "Running OpenVSLAM image ${vslam_id}" >> ${log_file}
            fi
        fi

        osgar_image_id=`docker image ls | grep rover | grep latest | awk '{print $3}'`
        srcp2_image_id=`docker image ls | grep comp | head -1 | awk '{print $3}'`
        echo "Running OSGAR image ${osgar_image_id}" >> ${log_file}
        echo "Running SRCP2 image ${srcp2_image_id}" >> ${log_file}

        timeout $TIMEOUT sh -c "( docker run --rm --network=host -t --name nasa-rover rover:latest /osgar/moon/docker/rover/run_solution.bash -r $ROUND &) | tee -a ${log_file} | grep -m 1 ${FAIL_SWITCH} -e \"${SUCCESS_STRING}\""
        # timeout exit status 124 means the timeout limit was reached
        timeout_status=$?
        grep -q "${SUCCESS_STRING}" < ${log_file}
        was_found=$?
        if [[ $timeout_status -eq 124 || $was_found -ne 0 || $UNSUCCESS_LOGS_ONLY -eq 0 ]]; then
            for f in $(docker exec -it nasa-rover /bin/bash -c "ls /*.log"); do
                f=`echo $f | sed 's/\r//g'`
                readarray -d . -t strarr <<< "$f"
                docker cp nasa-rover:$f ${log_dir}/${strarr[0]}-seed_${i}_run_${j}.${strarr[1]};
            done
        fi

        docker kill nasa-rover >& /dev/null

        if [[ $VSLAM -eq 1 ]]; then
            if [[ $ROUND -eq 2 ]]; then
                docker kill openvslam1 >& /dev/null
                docker kill openvslam2 >& /dev/null
            else
                docker kill openvslam >& /dev/null
            fi
        fi

        docker kill srcp2-simulation >& /dev/null
    done
done
