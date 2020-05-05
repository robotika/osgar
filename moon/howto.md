In one terminal start simulation:

git/srcp2-competitors$ docker/scripts/launch/roslaunch_docker --run-round 1 -n
(-n = no GUI does not work at the moment, because it does not start the simulation, i.e. this is "for later")

In the second terminal first build image:
git/osgar/moon/docker$ ./build.bash rover

and then run it:
docker run --network=host -it --rm rover:latest

Note, that you can also use interactive way via:
docker run --network=host -it --rm rover:latest /bin/bash

