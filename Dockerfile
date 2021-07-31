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
# orbslam2 - https://github.com/alsora/ros2-ORB_SLAM2
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
            ffmpeg \
            libglew-dev \
		  libboost-all-dev \
		  libboost-system-dev \
		  libcanberra-gtk-module \
            libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# pangolin
RUN git clone https://github.com/stevenlovegrove/Pangolin /tmp/pangolin && \
    cd /tmp/pangolin && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make -j$(nproc) && \
    make install 
    
# orbslam2 (use Windfisch fork for OpenCV4 support)
# note:  this does NOT use CUDA - see https://github.com/thien94/ORB_SLAM2_CUDA
ENV ORB_SLAM2_ROOT_DIR="/opt/ORB_SLAM2"
RUN git clone https://github.com/Windfisch/ORB_SLAM2 ${ORB_SLAM2_ROOT_DIR} && \
    cd ${ORB_SLAM2_ROOT_DIR} && \
    wget --no-check-certificate https://github.com/alsora/ros2-ORB_SLAM2/raw/master/docker/scripts/build.sh && \
    wget --no-check-certificate https://github.com/alsora/ros2-ORB_SLAM2/raw/master/docker/scripts/orbslam.patch && \
    git apply orbslam.patch && \
    bash build.sh 
    
# ros2_orbslam
RUN source ${ROS_ENVIRONMENT} && \
    cd ${WORKSPACE_ROOT} && \
    git clone https://github.com/alsora/ros2-ORB_SLAM2 src/ros2-ORB_SLAM2 && \
    colcon build --symlink-install --packages-select ros2_orbslam --event-handlers console_direct+
    
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
# note:  disable g2o when using orbslam, because orbslam has its own g2o
#RUN git clone https://github.com/RainerKuemmerle/g2o /tmp/g2o && \
#    cd /tmp/g2o && \
#    mkdir build && \
#    cd build && \
#    cmake -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_OPENGL=OFF .. && \
#    make -j$(nproc) && \
#    make install && \
#    rm -rf /tmp/g2o
    
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
RUN git clone https://github.com/introlab/rtabmap.git /opt/rtabmap && \
    cd /opt/rtabmap/build && \
    cmake -DWITH_PYTHON=ON -DWITH_ORB_SLAM=ON -DWITH_G2O=OFF -DWITH_TORCH=ON -DTorch_DIR=/usr/local/lib/python3.6/dist-packages/torch/share/cmake/Torch .. && \
    make -j$(nproc) && \
    make install

#
# Fix broken package.xml in test_pluginlib that crops up if/when rosdep is run again
#
#   Error(s) in package '/opt/ros/foxy/build/pluginlib/prefix/share/test_pluginlib/package.xml':
#   Package 'test_pluginlib' must declare at least one maintainer
#   The package node must contain at least one "license" tag
#
RUN TEST_PLUGINLIB_PACKAGE="${ROS_ROOT}/build/pluginlib/prefix/share/test_pluginlib/package.xml" && \
    sed -i '/<\/description>/a <license>BSD<\/license>' $TEST_PLUGINLIB_PACKAGE && \
    sed -i '/<\/description>/a <maintainer email="michael@openrobotics.org">Michael Carroll<\/maintainer>' $TEST_PLUGINLIB_PACKAGE && \
    cat $TEST_PLUGINLIB_PACKAGE

# since rtabmap_ros is an 'unreleased' package for ros2, manually pull it's dependencies from
# https://github.com/introlab/rtabmap_ros/blob/dfdbe1f68314e851e017c8af3788b17518a5000b/package.xml#L24
RUN source ${ROS_ENVIRONMENT} && \
    cd ${WORKSPACE_ROOT} && \
    rosinstall_generator --deps --exclude-path /opt/ros/${ROS_DISTRO}/src --rosdistro ${ROS_DISTRO} \
		laser_geometry \
		pcl_conversions \
		rviz_common \
		rviz_rendering \
		rviz_default_plugins \
	> ${ROS_DISTRO}.rtabmap.rosinstall && \
    cat ${ROS_DISTRO}.rtabmap.rosinstall && \
    vcs import src < ${ROS_DISTRO}.rtabmap.rosinstall && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv" && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \
    colcon build --symlink-install

ENV PYTORCH_PATH="/usr/local/lib/python3.6/dist-packages/torch"
ENV LD_LIBRARY_PATH="${PYTORCH_PATH}/lib:${LD_LIBRARY_PATH}"

# build rtabmap_ros, but first patch it to import tf2_geometry_msgs correctly
RUN source ${ROS_ENVIRONMENT} && \
    cd ${WORKSPACE_ROOT} && \
    git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros && \
    sed -i '/find_package(tf2_ros REQUIRED)/a find_package(tf2_geometry_msgs REQUIRED)' src/rtabmap_ros/CMakeLists.txt && \
    sed -i '/   tf2_ros/a tf2_geometry_msgs' src/rtabmap_ros/CMakeLists.txt && \
    cat src/rtabmap_ros/CMakeLists.txt && \
    colcon build --symlink-install --packages-select rtabmap_ros --event-handlers console_direct+
    
    
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
