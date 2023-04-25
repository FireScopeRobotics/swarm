#!/usr/bin/env bash

# Launch the robot
source /opt/ros/noetic/setup.bash 
source $HOME/swarm/devel/setup.bash

echo "Launching application, please wait!"
roslaunch fydp_mapping fydp_slam_toolbox_map.launch