#!/bin/bash

#
# make sure gazebo has been installed first
#
GAZEBO_DIR=~/.gazebo
GAZEBO_MODEL_DIR=$GAZEBO_DIR/models
GAZEBO_JETBOT_DIR=$GAZEBO_MODEL_DIR/jetbot

echo "checking for:  " $GAZEBO_DIR

if [ ! -d "$GAZEBO_DIR" ]; then
	echo "error:  please install and run Gazebo once before running this script"
	exit
fi

echo "checking for:  "  $GAZEBO_MODEL_DIR

if [ ! -d "$GAZEBO_MODEL_DIR" ]; then
	echo "error:  please install and run Gazebo once before running this script"
	exit
fi

echo "checking for:  " $GAZEBO_JETBOT_DIR

if [ -d "$GAZEBO_JETBOT_DIR" ]; then
	echo "error:  $GAZEBO_JETBOT_DIR already exists"
	exit
fi

#
# get the directory of the jetbot model
#
SOURCE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
JETBOT_DIR=$SOURCE_DIR/jetbot

echo "source from:   " $SOURCE_DIR
echo "jetbot from:   " $JETBOT_DIR
echo " "
echo "linking $GAZEBO_JETBOT_DIR -> $JETBOT_DIR" 

ln -s $JETBOT_DIR $GAZEBO_JETBOT_DIR
