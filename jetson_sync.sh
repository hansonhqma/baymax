#!/bin/bash

# Make sure to set environment variable JETSON_IP as applicable

set -e

ping $JETSON_IP -c 1

ssh jetson@$JETSON_IP "mkdir -p ~/baymax/catkin_ws/src"
rsync -r catkin_ws/src jetson@$JETSON_IP:~/baymax/catkin_ws
