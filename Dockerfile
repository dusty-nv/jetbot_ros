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
# install Gazebo
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
            cmake \
            libgazebo9-dev \
		  gazebo9 \
		  gazebo9-common \
		  gazebo9-plugin-base \
    && rm -rf /var/lib/apt/lists/*
    
RUN git clone https://github.com/dusty-nv/py3gazebo /opt/py3gazebo && \
    pip3 install protobuf>=2.6 --verbose && \
    pip3 install trollius --verbose && \
    RUN pip3 install pynput --verbose

ENV PYTHONPATH=/opt/py3gazebo

ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-9/models:/root/.gazebo/models:/jetbot_ros/gazebo
ENV GAZEBO_MASTER_URI=http://localhost:11346

