#!/usr/bin/env bash
#
# This script builds the jetbot_ros docker container from source.
# It should be run from the root dir of the jetbot_ros project:
#
#     $ cd /path/to/your/jetbot_ros
#     $ docker/build.sh
#
# Also you should set your docker default-runtime to nvidia:
#     https://github.com/dusty-nv/jetson-containers#docker-default-runtime
#
ROS_DISTRO=${1:-"foxy"}
BASE_IMAGE=$2

# break on errors
set -e

# find L4T_VERSION
source docker/tag.sh

if [ -z $BASE_IMAGE ]; then
	BASE_IMAGE="dustynv/ros:$ROS_DISTRO-slam-l4t-$TAG"
fi


build_container()
{
	local container_image=$1
	local dockerfile=$2
	
	echo "building $container_image"
	echo "BASE_IMAGE=$BASE_IMAGE"

	sudo docker build -t $container_image -f $dockerfile \
			--build-arg BASE_IMAGE=$BASE_IMAGE \
			.
}

build_container "jetbot_ros:$ROS_DISTRO-$TAG" "Dockerfile"
