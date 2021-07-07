#!/usr/bin/env bash

# find OS version
source docker/l4t_version.sh

TAG="r$L4T_VERSION"

if [ $L4T_VERSION = "32.5.1" ]; then
	TAG="r32.5.0"
fi	
	
#CONTAINER_IMAGE_ELOQUENT="jetbot_ros:eloquent-$TAG"
#CONTAINER_IMAGE_FOXY="jetbot_ros:foxy-$TAG"

