# Team Name
Robotika.cz

# Team Organization
Robotika.cz

# Team Location
Czech Republic

# Team Point of Contact
Martin Dlouhy

# POC Email
md-at-robotika.cz

# POC Phone
+420 604 377 599

# Team Website
https://robotika.cz/competitions/subtchallenge/

# List of Team members and Organizations
Martin Dlouhy, Robotika.cz
Jiri Isa,
Zbynek Winkler, Robotika.cz
Pavel Jiroutek,
Jakub Lev, Czech University of Life Science Prague

Pavel Skotak,
Milan Kroulik, Czech University of Life Science Prague
Stanislav Petrasek, Czech University of Life Science Prague
Frantisek Brabec


# Technical Approach

## Version 0 (2018-12-21, one artifact)

We have started with a single ground robot (X1) and the low end configuration
(X1_SENSOR_CONFIG_1=1) to test the basic functionality. Laser (with 5m range)
is used for obstacle avoidance and camera (RGB 320x240) for artifacts
recognition. From ROS we also use "/X1/imu/data" for yaw and pitch estimation
and "/X1/x1_velocity_controller/odom" for measuring distance traveled.

The version 0 (minimalistic version which can score) is using "follow wall"
algorithm with 1.5m desired distance, speed 1m/s and slowing down to 0.5m/s for
tight turns in dead-ends corners. Global 3D pose is calculated from IMU angles
and distance traveled. There is only one type of recognized artifact at the
moment (fire extinguisher) which is detected by its bright red color.

All simulations run on older Lenovo notebook with NVidia card, which is
probably not used with current ROS version (we had to switch laser sensor from
"gpu_ray" to "ray" and limit minimal distance to 0.5m in order to get
reasonable data). The OS was reinstalled to Ubuntu 18. We are using "no GUI"
option in order to speed up the simulation but it is still around 10x slower
than reality.

We are using Python and OSGAR (open source garden/generic autonomous robot)
library [1]. There is newly developed ROSProxy node, which handles
communication with ROS (XML RPC for callbacks and TCP connections for
individual ROS topics). All data are logged and timestamped on OSGAR side and
the logs are primary source for further development.  Some reference logs are
already available at [2]. OSGAR was used for real machines till now, so in
particular timestamps may need adaptation to slow simulation.

The next step is to identify other artifacts from already collected image data.
Also missing down-drop detector is critical (see video [3]). We asked for
combination of two LIDARs, which is still relatively low cost, when one is
pointing down or optionally mounted on gimbal so it can scan 3D on demand.

We plan to stick with simple navigation routines, i.e. follow left/right wall,
navigate on Voronoi diagram on tight spaces (if Vector Field Histogram fails),
recognize crossing and choose exit (for "islands" not explored by wall
following).  The drones will be the next step necessary due to down-drops and
places not reachable by ground vehicles. Due to limited time we did not
investigate available models yet, so it is hard to judge complexity. Probably
RGB-D sensor will be used for navigation.

The multi-robot cooperative task is planned for the second stage when basic
individual machines will fully autonomously navigate. The generation of global
map is at the moment in post-processing.  The detailed information about
current development state is available at [4] (at the moment in Czech, but we
will switch to English in case of interest).

[1] https://github.com/robotika/osgar/
[2] http://osgar.robotika.cz/
[3] https://youtu.be/jKeVcoSllNk?t=132
[4] https://robotika.cz/competitions/subtchallenge/


## Version 1 (2019-02-16, two artifacts)

The version 1 is extension of version 0 for two X2 robots (named X2L and X2R).
Both are navigating towards the first easy to recognize artifacts (red fire
extinguisher and red backpack). X2L runs version 0 following the left wall and
X2R the right wall. The artifacts are reported back at the Base Station area.


## Version 2 (2019-02-24, three artifacts)

The version 2 is follow-up of previous version with two X2 robots. The main
difference is precise artifact localization relative to the robot (critical for
the valve recognized by X2R) by combination of camera image and LIDAR scan.
Moreover a return path is optimized by pruning the trace and skipping
unnecessary dead-ends or loops.

The trial is no longer terminated with the first detection but by timeout.
Duplicities caused by multiple observations of the same artifact by the same
robot are filtered out.

More detailed info is available here:
https://robotika.cz/competitions/subtchallenge/stix/en#190223


## Version 3 (2019-03-03, four artifacts)

The searching time is set now to 10 minutes of simulation time, then the robots
return to base station. Newly electrical box is recognized and reported.
Because the box is invisible to LIDAR collision detector can terminate the
search.

Some screenshots from development are available here:
https://robotika.cz/competitions/subtchallenge/stix/en#190303


## Version 4 (2019-03-08, five artifacts)

Increased the simulation time to whole 20 minutes, detection of red
toolbox and object avoidance for white electrical box visible only
on camera.


## Version 5 (2019-03-09, seven artifacts)

After adding a fuzzy local path planner, robots can get through narrow passages
and explore new parts of the tunnel system.


## Version 6 (2019-03-13, eight artifacts)

Added detection of radio artifact.

