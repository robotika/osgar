# Ubuntu 18.04 with nvidia-docker2 beta opengl support
FROM nvidia/opengl:1.0-glvnd-devel-ubuntu18.04 as stage1

RUN apt-get update -qq \
 && apt-get install -y -qq \
        apt-utils \
        bash-completion \
        build-essential \
        cmake \
        cppcheck \
        gdb \
        git \
        iputils-ping \
        libarchive-tools \
        lsb-release \
        mc \
        net-tools \
        python3-pip \
        python3-venv \
        software-properties-common \
        sudo \
        vim \
        wget \
 && apt-get clean -qq

# set default timezone
RUN export DEBIAN_FRONTEND=noninteractive \
 && apt-get update -qq \
 && apt-get install -y -qq \
    tzdata \
 && ln -fs /usr/share/zoneinfo/Europe/Prague /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && apt-get clean -qq

# install ROS and required packages
RUN /bin/sh -c 'echo "deb [trusted=yes] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && apt-get update -qq \
 && apt-get install -y -qq \
    python-catkin-tools \
    python-rosdep \
    python-rosinstall \
    ros-melodic-desktop \
    ros-melodic-joystick-drivers \
    ros-melodic-pointcloud-to-laserscan \
    ros-melodic-robot-localization \
    ros-melodic-rotors-control \
    ros-melodic-rviz-imu-plugin \
    ros-melodic-spacenav-node \
    ros-melodic-tf2-sensor-msgs \
    ros-melodic-twist-mux \
 && rosdep init \
 && apt-get clean -qq

# sdformat8-sdf conflicts with sdformat-sdf installed from gazebo
# so we need to workaround this using a force overwrite
# Do this before installing ign-gazebo
RUN /bin/sh -c 'echo "deb [trusted=yes] http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && /bin/sh -c 'wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -' \
 && /bin/sh -c 'apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'

# install ign-blueprint
RUN apt-get update -qq \
&&  apt-get install -y -qq \
    ignition-blueprint \
    ros-melodic-ros-ign \
 && apt-get clean -qq

SHELL ["/bin/bash", "-c"]

FROM stage1 as build
WORKDIR /tmp
RUN mkdir subt-ws \
  && cd subt-ws \
  && git clone --depth=1 https://github.com/osrf/subt.git src \
  && source /opt/ros/melodic/setup.bash \
  && catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/subt -DCMAKE_BUILD_TYPE=Release

FROM stage1

COPY --from=build /opt/subt/ /opt/subt/
COPY --from=build /tmp/subt-ws/src/.git/refs/heads/master /opt/subt/HEAD

WORKDIR /osgar-ws

RUN /usr/bin/python3 -m venv env && \
  /osgar-ws/env/bin/pip install --no-cache-dir \
  "msgpack==1.0.0" \
  "numpy==1.18.2" \
  "opencv-python==4.2.0.32" \
  "pyzmq==19.0.0"
