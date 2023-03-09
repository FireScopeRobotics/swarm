#!/usr/bin/env bash

# Launch the robot
source /opt/ros/noetic/setup.bash 
source /home/ayushg/swarm/devel/setup.bash

echo "Launching application, please wait!"
roslaunch explore_lite explore_slam_costmap_multi_robot_test.launch