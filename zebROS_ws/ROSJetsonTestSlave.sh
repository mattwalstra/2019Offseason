#!/usr/bin/env bash

# Setup ROS for Jetson Slave
source /opt/ros/melodic/setup.bash
source /home/ubuntu/2019Offseason/zebROS_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.70.54.12:5802
export ROS_IP=`ip route get 10.70.54.1 | sed 's/ via [[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+//' | sed 's/lo //' | head -1 | cut -d ' ' -f 5`
export ROSLAUNCH_SSH_UNKNOWN=1
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

exec "$@"
