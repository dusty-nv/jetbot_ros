#!/usr/bin/env bash

# find container tag from L4T version
source docker/tag.sh

REGISTRY="dustynv"

# push image
push() 
{
	local local_container=$1
	local remote_container=$REGISTRY/$local_container
	
	if [[ "$(sudo docker images -q $local_container 2> /dev/null)" == "" ]]; then
		echo "couldn't find local image $local_container -- skipping push"
		return
	fi
	
	sudo docker rmi $remote_container
	sudo docker tag $local_container $remote_container
	
	echo "pushing image $remote_container"
	sudo docker push $remote_container
	echo "done pushing image $remote_container"
}

push "jetbot_ros:eloquent-$TAG"
push "jetbot_ros:foxy-$TAG"