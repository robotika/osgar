FROM nvidia/cuda:11.2.2-cudnn8-runtime-ubuntu18.04 as stage1

RUN apt-get update \
 && apt-get install -y \
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
 && apt-get clean

# set default timezone
RUN export DEBIAN_FRONTEND=noninteractive \
 && apt-get update \
 && apt-get install -y \
    tzdata \
 && ln -fs /usr/share/zoneinfo/Europe/Prague /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && apt-get clean

# install ROS and required packages
RUN /bin/sh -c 'echo "deb [trusted=yes] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && apt-get update \
 && apt-get install -y \
    libgoogle-glog0v5 \
    python-catkin-tools \
    python-laser-geometry \
    python-rosdep \
    python-rosinstall \
    ros-melodic-desktop \
    ros-melodic-joystick-drivers \
    ros-melodic-octomap-rviz-plugins \
    ros-melodic-octomap-server \
    ros-melodic-pointcloud-to-laserscan \
    ros-melodic-robot-localization \
    ros-melodic-rotors-control \
    ros-melodic-rtabmap-ros \
    ros-melodic-rviz-imu-plugin \
    ros-melodic-spacenav-node \
    ros-melodic-tf2-sensor-msgs \
    ros-melodic-twist-mux \
    ros-melodic-control-toolbox \
    ros-melodic-controller-manager \
    ros-melodic-diff-drive-controller \
    ros-melodic-multimaster-launch \
    ros-melodic-joint-state-controller \
    ros-melodic-joint-trajectory-controller \
 && rosdep init \
 && apt-get clean

# add gazebo package repository
RUN /bin/sh -c 'echo "deb [trusted=yes] http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && /bin/sh -c 'wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -' \
 && /bin/sh -c 'apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'

# install ign-blueprint and libignition-launch3-dev
RUN apt-get update \
&&  apt-get install -y \
    ignition-blueprint \
    libignition-launch3-dev \
    ros-melodic-ros-ign \
 && apt-get clean

SHELL ["/bin/bash", "-c"]

# build osrf/subt repository
# ... TODO one day:
#  && rosdep update \
#  && rosdep install --from-paths src --ignore-src -r \
FROM stage1 as build
WORKDIR /tmp
RUN mkdir subt-ws \
  && cd subt-ws \
  && git clone --depth=1 https://github.com/osrf/subt.git src \
  && source /opt/ros/melodic/setup.bash \
  && catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/subt -DCMAKE_BUILD_TYPE=Release

FROM stage1

# copy the binary packages of osrf/subt
COPY --from=build /opt/subt/ /opt/subt/
COPY --from=build /tmp/subt-ws/src/.git/refs/heads/master /opt/subt/HEAD

WORKDIR /osgar-ws

RUN /usr/bin/python3 -m venv env

RUN ./env/bin/pip install --no-cache-dir --upgrade pip wheel setuptools

RUN ./env/bin/pip install --no-cache-dir \
  "msgpack==1.0.0" \
  "numpy==1.19.4" \
  "protobuf==3.13.0" \
  "pyzmq==19.0.2"

RUN ./env/bin/pip install --no-cache-dir \
  "torch==1.8.0+cu111" -f https://download.pytorch.org/whl/torch_stable.html

RUN ./env/bin/pip install --no-cache-dir \
  http://osgar.robotika.cz/subt/pip/opencv/opencv_contrib_python-4.5.2.52-cp36-cp36m-linux_x86_64.whl

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,utility

RUN apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python-pip python-wheel \
  && apt-get clean \
  && pip2 install --no-cache-dir --user \
    "msgpack==1.0.0" \
    "pyzmq==19.0.2"

# disable crazy threading behavior inside openblas (and maybe also in opencv)
# https://github.com/xianyi/OpenBLAS#setting-the-number-of-threads-using-environment-variables
ENV OMP_NUM_THREADS=1
