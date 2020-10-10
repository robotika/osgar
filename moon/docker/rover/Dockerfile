FROM ros:melodic

ENV DEBIAN_FRONTEND noninteractive

# install dependencies via apt
ENV DEBCONF_NOWARNINGS yes

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-melodic-catkin \
    ros-melodic-cv-bridge \
    ros-melodic-tf2-geometry-msgs \
    ros-melodic-image-transport \
    && rm -rf /var/lib/apt/lists/*

RUN set -x && \
  apt-get update -y -qq && \
  apt-get upgrade -y -qq --no-install-recommends && \
  : "basic dependencies" && \
  apt-get install -y -qq \
    build-essential \
    pkg-config \
    cmake \
    git \
    wget \
    less \
    curl \
    vim \
    tar \
    unzip && \
  : "ROS dependencies" && \
  apt-get install -y -qq \
    python && \
  : "g2o dependencies" && \
  apt-get install -y -qq \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libglew-dev && \
  : "OpenCV dependencies" && \
  apt-get install -y -qq \
    libgtk-3-dev \
    libjpeg-dev \
    libpng++-dev \
    libtiff-dev \
    libopenexr-dev \
    libwebp-dev \
    ffmpeg \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    libavresample-dev && \
  : "other dependencies" && \
  apt-get install -y -qq \
    libyaml-cpp-dev && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

ARG CMAKE_INSTALL_PREFIX=/usr/local
ARG NUM_THREADS=1

ENV CPATH=${CMAKE_INSTALL_PREFIX}/include:${CPATH}
ENV C_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include:${C_INCLUDE_PATH}
ENV CPLUS_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include:${CPLUS_INCLUDE_PATH}
ENV LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:${LIBRARY_PATH}
ENV LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:${LD_LIBRARY_PATH}

# Eigen
ARG EIGEN3_VERSION=3.3.7
WORKDIR /tmp
RUN set -x && \
  wget -q https://gitlab.com/libeigen/eigen/-/archive/${EIGEN3_VERSION}/eigen-${EIGEN3_VERSION}.tar.bz2 && \
  tar xf eigen-${EIGEN3_VERSION}.tar.bz2 && \
  rm -rf eigen-${EIGEN3_VERSION}.tar.bz2 && \
  cd eigen-${EIGEN3_VERSION} && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *
ENV Eigen3_DIR=${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake

# g2o
ARG G2O_COMMIT=9b41a4ea5ade8e1250b9c1b279f3a9c098811b5a
WORKDIR /tmp
RUN set -x && \
  git clone https://github.com/RainerKuemmerle/g2o.git && \
  cd g2o && \
  git checkout ${G2O_COMMIT} && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_UNITTESTS=OFF \
    -DBUILD_WITH_MARCH_NATIVE=OFF \
    -DG2O_USE_CHOLMOD=OFF \
    -DG2O_USE_CSPARSE=ON \
    -DG2O_USE_OPENGL=OFF \
    -DG2O_USE_OPENMP=ON \
    -DG2O_BUILD_APPS=OFF \
    -DG2O_BUILD_EXAMPLES=OFF \
    -DG2O_BUILD_LINKED_APPS=OFF \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *
ENV g2o_DIR=${CMAKE_INSTALL_PREFIX}/lib/cmake/g2o

# OpenCV
ARG OPENCV_VERSION=3.4.10
WORKDIR /tmp
RUN set -x && \
  wget -q https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip && \
  unzip -q ${OPENCV_VERSION}.zip && \
  rm -rf ${OPENCV_VERSION}.zip && \
  cd opencv-${OPENCV_VERSION} && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    -DBUILD_DOCS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_JASPER=OFF \
    -DBUILD_OPENEXR=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_opencv_apps=OFF \
    -DBUILD_opencv_dnn=OFF \
    -DBUILD_opencv_ml=OFF \
    -DBUILD_opencv_python_bindings_generator=OFF \
    -DENABLE_CXX11=ON \
    -DENABLE_FAST_MATH=ON \
    -DWITH_EIGEN=ON \
    -DWITH_FFMPEG=ON \
    -DWITH_OPENMP=ON \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *
ENV OpenCV_DIR=${CMAKE_INSTALL_PREFIX}/lib/cmake/opencv

# DBoW2
ARG DBOW2_COMMIT=687fcb74dd13717c46add667e3fbfa9828a7019f
WORKDIR /tmp
RUN set -x && \
  git clone https://github.com/shinsumicco/DBoW2.git && \
  cd DBoW2 && \
  git checkout ${DBOW2_COMMIT} && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *
ENV DBoW2_DIR=${CMAKE_INSTALL_PREFIX}/lib/cmake/DBoW2

# protobuf
WORKDIR /tmp
RUN set -x && \
  apt-get update -y -qq && \
  apt-get upgrade -y -qq --no-install-recommends && \
  apt-get install -y -qq autogen autoconf libtool && \
  wget -q https://github.com/google/protobuf/archive/v3.6.1.tar.gz && \
  tar xf v3.6.1.tar.gz && \
  cd protobuf-3.6.1 && \
  ./autogen.sh && \
  ./configure --prefix=${CMAKE_INSTALL_PREFIX} --enable-static=no && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf * && \
  apt-get purge -y -qq autogen autoconf libtool && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

# OpenVSLAM
WORKDIR /
ARG OPENVSLAM_COMMIT=026827ebd59825de42463f853e910e9ccb44f0ef
RUN set -x && \
  git clone https://github.com/frantisekbrabec/openvslam.git && \
  cd openvslam && \
  git checkout ${OPENVSLAM_COMMIT} && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DBUILD_WITH_MARCH_NATIVE=OFF \
    -DUSE_PANGOLIN_VIEWER=OFF \
    -DUSE_SOCKET_PUBLISHER=OFF \
    -DUSE_STACK_TRACE_LOGGER=ON \
    -DBOW_FRAMEWORK=DBoW2 \
    -DBUILD_TESTS=OFF \
    .. && \
  make -j${NUM_THREADS} && \
  rm -rf CMakeCache.txt CMakeFiles Makefile cmake_install.cmake example src && \
  chmod -R 777 ./*

# Copy SRCP2 env to workspace
COPY ./srcp2-competitors /srcp2-competitors/

# OpenVSLAMROS
WORKDIR /openvslam/ros/1/
RUN /bin/bash -c 'source /srcp2-competitors/ros_workspace/install/setup.bash; cd /openvslam/ros/1/; catkin_make \
    -DBUILD_WITH_MARCH_NATIVE=OFF \
    -DUSE_PANGOLIN_VIEWER=OFF \
    -DUSE_SOCKET_PUBLISHER=OFF \
    -DUSE_STACK_TRACE_LOGGER=ON \
    -DBOW_FRAMEWORK=DBoW2'

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y python-pip python3-pip openssh-client vim less
RUN python -m pip install pyzmq

RUN /usr/bin/python3 -m pip install scikit-build
RUN /usr/bin/python3 -m pip install msgpack pyzmq shapely scipy
RUN /usr/bin/python3 -m pip install --install-option="-j$(nproc)" "opencv-python>=3.4.10,<3.4.11"

RUN mkdir -p /rover_workspace/src

# Alien Control
WORKDIR /rover_workspace/src
RUN set -x && \
  git clone https://github.com/frantisekbrabec/aliencontrol.git

COPY ./osgar/moon/ros_launchfiles /rover_workspace/src/aliencontrol/launch
RUN chmod 777 /rover_workspace

RUN rm -rf /rover_workspace/devel
RUN rm -rf /rover_workspace/build

RUN  /bin/bash -c "source /openvslam/ros/1/devel/setup.bash; cd /rover_workspace; catkin_make"

RUN echo "source /rover_workspace/devel/setup.bash" >> /root/.bashrc

COPY ./osgar /osgar/

RUN rm -rf /osgar/.git
RUN rm -rf /osgar/.github
RUN rm -rf /osgar/research_scripts
RUN rm -rf /osgar/disparity_images

WORKDIR /
ENTRYPOINT ["/osgar/moon/docker/rover/entrypoint.bash"]
CMD ["/bin/bash"]
