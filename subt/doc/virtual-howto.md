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

## Precoditions

3D X server needs to be running under the same user that is going to be running the simulation.
If the X server runs display `:0` the variable `DISPLAY` needs to be set and exported
```
export DISPLAY=:0
```
Also `XAUTHORITY` variable needs to be set to point to the location of the XAuthority file. Example on
ubuntu could be
```
$ echo $XAUTHORITY
/run/user/1000/gdm/Xauthority
```
Another possiblity is to run `xhost +` to disable the security on the X server.

The main point here is that the simulation can find and access the 3D X server.

## Build and run locally simple unittest image
```
./subt/docker/build.bash unittest
./subt/docker/run.bash unittest
```
It is necessary to run also simulation and bridge in other two terminals:
- terminal 1
    ```
    ROBOT=X100L WORLD=simple_cave_01 ./subt/script/sim.bash
    ```

- terminal 2
    ```
    ROBOT=X100L WORLD=simple_cave_01 ./subt/script/bridge.bash
    ```

Note, that configuration and robot name is variable. The command above with
robot name X100L encodes exploring for 100 s and navigating
along a wall on the left. Our own ROBOTIKA_X2_SENSOR_CONFIG_1 is used by default. It is
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
which by default runs `/osgar-ws/run_solution.bash` which itself is soft link to
`/osgar-ws/src/osgar/subt/docker/robotika/run_solution.bash` inside the container.
To get a shell inside the container instead, run
```commandline
./subt/docker/run.bash robotika bash
```
and you can call `run_solution.bash` when you are ready.

A copy of `osgar` directory from the time of the build of the image is located
at `/osgar-ws/src/osgar/`. For local development it is advantageous
to mount your `osgar` directory from the host over this directory in the container.

```commandline
./subt/script/devel.bash
```

When you do so, you can edit the files as you are used to. To rebuild the ROS
nodes from within the running container, call `make` in `/osgar-ws/` directory
After that running `/osgar-ws/run_solution.bash` will run the rebuilt version.

At this moment you should see waiting for ROS master, debug count outputs of received messages
(similarly as in unittest) and Python3 outputs of robot navigating towards the gate. The exploration reports
number and type of received messages by OSGAR.

There is a logfile available when the robot finishes in `/osgar-ws/logs/`.
It is necessary to copy the logfile (for example
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
