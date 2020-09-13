#!/usr/bin/env bash

RUN_UNPAUSED=0

while getopts uhr: arg; do
    case $arg in
	r)
	    case $OPTARG in
		"1" )
		    JSONFILES=("moon-round1.json")
		    ROVERSCRIPTS=("rospy_scout_round1.py --robot_name=scout_1 --push_port=5555 --pull_port=5556 --reqrep_port=5557")
                    DAEMONS=("rosrun openvslam run_slam -v /openvslam/orb_vocab.dbow2 -c /openvslam/rover_camera_config.yaml -r scout_1")
		    ;;
		"2" )
		    JSONFILES=("moon-excavator-round2.json" "moon-hauler-round2.json")
		    ROVERSCRIPTS=("rospy_excavator_round2.py --robot_name=excavator_1 --push_port=5555 --pull_port=5556 --reqrep_port=5557" "rospy_hauler_round2.py --robot_name=hauler_1 --push_port=6555 --pull_port=6556 --reqrep_port=6557")
                    DAEMONS=("rosrun openvslam run_slam -v /openvslam/orb_vocab.dbow2 -c /openvslam/rover_camera_config.yaml -r excavator_1")
		    ;;
		"3" )
		    JSONFILES=("moon-round3.json")
		    ROVERSCRIPTS=("rospy_scout_round3.py --robot_name=scout_1 --push_port=5555 --pull_port=5556 --reqrep_port=5557")
                    DAEMONS=()
		;;

	    esac
	    ;;
	h)
	    echo "-r [1|2|3] to choose the run to execute"
	    echo "-u run unpaused"
	    exit
	    ;;
        u)
            RUN_UNPAUSED=1
            ;;
	*)
	    exit
	    ;;
    esac
done

if [[ $RUN_UNPAUSED -eq 1 ]]; then
    echo "Unpause simulation in 5 sec"
    ( sleep 5; rosservice call /gazebo/unpause_physics "{}" ) &
fi

ROBOT_PIDS=()
ROS_PIDS=()
DAEMON_PIDS=()

echo "Start robot solution (no timeout)"
export OSGAR_LOGS=`pwd`
cd /osgar
for s in ${JSONFILES[@]}; do
    echo "starting recording of $s"
    python3 -m osgar.record moon/config/$s --note "collect some ROS data" &
    ROBOT_PIDS+=($!)
done


for ((i=0; i < ${#DAEMONS[@]}; i++)) do
    echo "Starting daemon '${DAEMONS[$i]}'"
    ${DAEMONS[$i]} &
    DAEMON_PIDS+=($!)
done


cd ..

# get directory where this bash script lives
samedir=$(dirname $(readlink -f "${BASH_SOURCE[0]}"))

# enable ROS DEBUG output to see if messages are being dropped
export ROSCONSOLE_CONFIG_FILE="${samedir}/rosconsole.config"

# Run your solution and wait for ROS master
# http://wiki.ros.org/roslaunch/Commandline%20Tools#line-45
## roslaunch subt_seed x1.launch --wait &

for ((i=0; i < ${#ROVERSCRIPTS[@]}; i++)) do
    echo "Starting script '${ROVERSCRIPTS[$i]}'"
    python /osgar/moon/rospy/${ROVERSCRIPTS[$i]} &
    ROS_PIDS+=($!)
done

# Turn everything off in case of CTRL+C and friends.
function shutdown {
    for p in ${ROBOT_PIDS[@]}; do
	kill $p
    done
    for p in ${DAEMON_PIDS[@]}; do
	kill $p
    done
    for p in ${ROS_PIDS[@]}; do
	kill $p
    done

    wait
    exit
}
trap shutdown SIGHUP SIGINT SIGTERM


# Wait for the controllers to finish.
for r in ${ROBOT_PIDS[@]}; do
    wait $r
done


echo "Sleep and finish"
sleep 30

# Take robot simulation down.
for r in ${ROS_PIDS[@]}; do
    wait $r
done
wait
