# SubT Challenge - Virtual Track HowTo

Here are instructions how to build, run locally and deploy to AWS "robotika" solution docker.
It is expected that you already have docker installed and your PC has compatible NVidia GPU.
For official instructions how to start with Docker please look at:
  https://bitbucket.org/osrf/subt/wiki/tutorials/SystemSetupDockerInstall


## Prepare OSGAR build environment
The convenience scripts for build and run are merged into OSGAR:
```
git clone https://github.com/robotika/osgar.git .
cd osgar
```

## Download official docker images
The currently available images are listed at:
  https://hub.docker.com/r/osrf/subt-virtual-testbed/tags
Note, that you will need cloudsim_sim_latest and cloudsim_bridge_latest.
```
docker pull osrf/subt-virtual-testbed:cloudsim_sim_latest
docker pull osrf/subt-virtual-testbed:cloudsim_bridge_latest
```

## Build and run locally simple unittest image
```
./subt/docker/build.bash unittest
./subt/docker/run.bash unittest
```

It is necessary to run also simulation and bridge in other two terminals:
- terminal 1
    ```
    xhost +local:root
    ROBOT=X0F200L WORLD=urban_circuit_practice_01 ./subt/script/sim.bash
    ```
    Note, that `xhost` workaround is not secure and it is current workaround how to start the process for the first time,
    open screen session and then use the simulator remotely via ssh.

- terminal 2
    ```
    ROBOT=X0F200L WORLD=urban_circuit_practice_01 ./subt/script/bridge.bash
    ```

Note, that configuration and robot name is variable. The command above with
robot name X0F200L encodes waiting for 0 s, exploring for 200 s and navigating
along a wall on the left. Our own ROBOTIKA_X2_SENSOR_CONFIG_1 is used. It is
a small robot, 30m lidar, 640x380 RGBD camera and gas detector.

The unittest should display number of received messages for scan, image, imu, odometry and clock. After 10000 clock
messages it publishes `/subt/finish` causing end of session on AWS. Locally user has to stop it manually via Ctrl+C.

It is necessary to kill & start again both sim and bridge docker for each run. Note, that there exists "docker_compose"
solution not covered by this HOWTO.


## Working with the main "robotika" image

### Build
```
./subt/docker/build.bash robotika
```

### Run locally

```commandline
./subt/docker/run.bash robotika
```
which by default runs `./src/osgar/subt/docker/robotika/run_solution.bash` inside the container.
To get a shell inside the docker instead, run
```commandline
./subt/docker/run.bash robotika bash
```
and you can call `run_solution.bash` when you are ready.

A copy of `osgar` directory from the time of the build of the image is located
at `/home/developer/subt_solution/osgar`. For local development it is advantageous
to mount your `osgar` directory from the host over this directory in the container.

```commandline
./subt/script/devel.bash
```

When you do so, you can edit the files as you are used to. To rebuild the ROS
nodes from within the running container, switch to `/osgar-ws/build/` directory
and call `make`. After that running
`./src/osgar/subt/docker/robotika/run_solution.bash` will run the rebuilt version.

At this moment you should see waiting for ROS master, debug count outputs of received messages
(similarly as in unittest) and Python3 outputs of robot navigating towards the gate. The exploration reports
number and type of received messages by OSGAR.

There is a logfile available when the robot finishes. It is in the current
directory with name zmq*.log. It is necessary to upload the logfile (for example
via `docker cp`) to the host for further analysis, otherwise
it will be lost with the termination of the container.


## Run in cloudsim

The "robotika" tags used for AWS are in the form "verXXX", where XXX is the release/build number.

```
./subt/docker/build.bash robotika
docker tag robotika 200670743174.dkr.ecr.us-east-1.amazonaws.com/subt/robotika:verXXX
eval `aws ecr get-login --no-include-email`
docker push 200670743174.dkr.ecr.us-east-1.amazonaws.com/subt/robotika:verXXX
```

## Extraction of OSGAR logfiles from ROS bag

CloudSim supports storing topic /robot_data into ROS bag (up to 2GB). There is helper tool for extraction:
```
python -m subt.rosbag2log <downloaded tar file>
```

Newly extracted file is named by tar file, i.e. `ver41p3-91c332d3-2066-466c-a9b9-e3418bbeb0a9-A10F900L.tar creates`
new file named `aws-ver41p3-A10F900L.log`
