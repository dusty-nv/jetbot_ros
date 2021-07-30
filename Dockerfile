# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#
# Build this Dockerfile by running the following commands:
#
#     $ cd /path/to/your/jetbot_ros
#     $ docker/build.sh
#
# Also you should set your docker default-runtime to nvidia:
#     https://github.com/dusty-nv/jetson-containers#docker-default-runtime
#

ARG BASE_IMAGE=dustynv/ros:foxy-ros-base-pytorch-l4t-r32.5.0
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"] 
ENV SHELL /bin/bash

ENV DEBIAN_FRONTEND=noninteractive
ARG MAKEFLAGS=-j$(nproc)
ENV LANG=en_US.UTF-8 
ENV PYTHONIOENCODING=utf-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

WORKDIR /tmp


#
# install utilities
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		  nano \
		  xterm \
		  lxterminal \
		  blender \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN git clone https://github.com/dusty-nv/py3gazebo /opt/py3gazebo && \
    pip3 install protobuf>=2.6 --verbose && \
    pip3 install trollius --verbose && \
    pip3 install pynput --verbose

ENV PYTHONPATH=/opt/py3gazebo
   
   
#
# JetBot hw controllers
#
RUN pip3 install Adafruit-MotorHAT Adafruit-SSD1306 --verbose


#
# environment setup
#   
ENV WORKSPACE_ROOT=/workspace
ENV JETBOT_ROOT=${WORKSPACE_ROOT}/src/jetbot_ros
ARG ROS_ENVIRONMENT=${ROS_ROOT}/install/setup.bash

ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-9/models:/root/.gazebo/models:${JETBOT_ROOT}/gazebo/models
ENV GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${JETBOT_ROOT}/gazebo/plugins/build/:/usr/local/lib/
ENV GAZEBO_MASTER_URI=http://localhost:11346

# setup workspace
WORKDIR ${WORKSPACE_ROOT}
RUN mkdir -p ${WORKSPACE_ROOT}/src

COPY scripts/setup_workspace.sh ${WORKSPACE_ROOT}/setup_workspace.sh
ENV PYTHONPATH="${JETBOT_ROOT}:${PYTHONPATH}"


#
# rtabmap - https://github.com/introlab/rtabmap_ros/tree/ros2
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		  libpcl-dev \
		  libpython3-dev \
		  python3-dev \
		  software-properties-common \
		  apt-transport-https \
		  ca-certificates \
		  gnupg \
		  libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# upgrade cmake - https://stackoverflow.com/a/56690743
RUN wget -qO - https://apt.kitware.com/keys/kitware-archive-latest.asc | apt-key add - && \
    apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main' && \
    apt-get update && \
    apt-get install -y --only-upgrade --no-install-recommends \
            cmake \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean \
    && cmake --version
    
# install recommended dependencies - https://github.com/introlab/rtabmap/wiki/Installation#dependencies 
# TODO build https://github.com/DrTimothyAldenDavis/SuiteSparse from source for CUDA if any advantage?
RUN git clone https://github.com/RainerKuemmerle/g2o /tmp/g2o && \
    cd /tmp/g2o && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_OPENGL=OFF .. && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/g2o
    
RUN git clone https://github.com/borglab/gtsam /tmp/gtsam && \
    cd /tmp/gtsam && \
    mkdir build && \
    cd build && \
    cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_WITH_TBB=OFF .. && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/gtsam
    
RUN git clone https://github.com/ethz-asl/libnabo /tmp/libnabo && \
    cd /tmp/libnabo && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/libnabo
    
RUN git clone https://github.com/ethz-asl/libpointmatcher /tmp/libpointmatcher && \
    cd /tmp/libpointmatcher && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/libpointmatcher

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
	libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# build rtabmap / rtabmap_ros
RUN source ${ROS_ENVIRONMENT} && \
    cd ${WORKSPACE_ROOT}/src && \
    git clone https://github.com/introlab/rtabmap.git rtabmap && \
    git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git rtabmap_ros && \
    cd ../ && \
    colcon build --symlink-install --event-handlers console_direct+ \
			  --cmake-args -DWITH_PYTHON=ON -DWITH_TORCH=ON -DTorch_DIR=/usr/local/lib/python3.6/dist-packages/torch/share/cmake/Torch
    
    
#
# ros_deep_learning package
#
RUN source ${ROS_ENVIRONMENT} && \
    cd ${WORKSPACE_ROOT}/src && \
    git clone https://github.com/dusty-nv/ros_deep_learning && \
    cd ../ && \
    colcon build --symlink-install --event-handlers console_direct+


#
# build project
#
COPY jetbot_ros ${JETBOT_ROOT}/jetbot_ros
COPY launch ${JETBOT_ROOT}/launch
COPY gazebo ${JETBOT_ROOT}/gazebo
COPY resource ${JETBOT_ROOT}/resource

COPY package.xml ${JETBOT_ROOT}
COPY setup.py ${JETBOT_ROOT}
COPY setup.cfg ${JETBOT_ROOT}

RUN cd ${JETBOT_ROOT}/gazebo/plugins/ && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make -j$(nproc) && \
    make install
    
RUN source ${ROS_ENVIRONMENT} && \
    cd ${WORKSPACE_ROOT} && \
    colcon build --symlink-install --event-handlers console_direct+


#
# setup entrypoint
#
COPY scripts/ros_entrypoint.sh /ros_entrypoint.sh

RUN sed -i \
    's/ros_env_setup="\/opt\/ros\/$ROS_DISTRO\/setup.bash"/ros_env_setup="${ROS_ROOT}\/install\/setup.bash"/g' \
    /ros_entrypoint.sh && \
    cat /ros_entrypoint.sh

RUN echo 'source ${ROS_ROOT}/install/setup.bash' >> /root/.bashrc && \
    echo 'source ${WORKSPACE_ROOT}/install/local_setup.bash' >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
