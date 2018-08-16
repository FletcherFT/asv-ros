#!/usr/bin/env bash
source /opt/ros/kinetic/setup.bash
source $HOME/ROS_ws/devel/setup.bash
export ROS_MASTER_URI=http://asv-server.local:11311
export ROS_HOSTNAME=$HOSTNAME.local
exec "$@"