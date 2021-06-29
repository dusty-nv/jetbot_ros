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

ARG BASE_IMAGE=nvcr.io/nvidia/l4t-pytorch:r32.5.0-pth1.6-py3
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
ENV LANG=en_US.UTF-8 
ENV PYTHONIOENCODING=utf-8

WORKDIR jetbot_ros

#
# install Gazebo and Blender
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
            cmake \
            libgazebo9-dev \
		  gazebo9 \
		  gazebo9-common \
		  gazebo9-plugin-base \
		  blender \
		  python3-opencv \
    && rm -rf /var/lib/apt/lists/*
    
RUN git clone https://github.com/dusty-nv/py3gazebo /opt/py3gazebo && \
    pip3 install protobuf>=2.6 --verbose && \
    pip3 install trollius --verbose && \
    pip3 install pynput --verbose

ENV PYTHONPATH=/opt/py3gazebo

ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-9/models:/root/.gazebo/models:/jetbot_ros/gazebo
ENV GAZEBO_MASTER_URI=http://localhost:11346


#
# install ROS2
#
ARG ROS_PKG=ros_base
ENV ROS_DISTRO=eloquent
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

# change the locale from POSIX to UTF-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# add the ROS deb repo to the apt sources list
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
          git \
		cmake \
		build-essential \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install ROS packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		ros-eloquent-`echo "${ROS_PKG}" | tr '_' '-'` \
		ros-eloquent-launch-xml \
		ros-eloquent-launch-yaml \
		ros-eloquent-vision-msgs \
		libpython3-dev \
		python3-colcon-common-extensions \
		python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# init/update rosdep
RUN apt-get update && \
    cd ${ROS_ROOT} && \
    rosdep init && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*

# compile yaml-cpp-0.6, which some ROS packages may use (but is not in the 18.04 apt repo)
RUN git clone --branch yaml-cpp-0.6.0 https://github.com/jbeder/yaml-cpp yaml-cpp-0.6 && \
    cd yaml-cpp-0.6 && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_SHARED_LIBS=ON .. && \
    make -j$(nproc) && \
    cp libyaml-cpp.so.0.6.0 /usr/lib/aarch64-linux-gnu/ && \
    ln -s /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.6.0 /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.6 && \
    rm -rf yaml-cpp-0.6

# gazebo_ros packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		ros-eloquent-gazebo-ros \
		ros-eloquent-gazebo-msgs \
		ros-eloquent-gazebo-ros-pkgs \
		ros-eloquent-gazebo-plugins \
		ros-eloquent-turtlebot3* \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean
    
# setup entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
