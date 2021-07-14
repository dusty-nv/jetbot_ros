#!/usr/bin/env bash

# find container tag from L4T version
source docker/tag.sh

REGISTRY="dustynv"

sudo docker pull $REGISTRY/jetbot_ros:eloquent-$TAG
sudo docker pull $REGISTRY/jetbot_ros:foxy-$TAG
