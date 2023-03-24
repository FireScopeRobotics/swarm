#!/usr/bin/env bash

# Launch the robot
source /opt/ros/noetic/setup.bash 
source /home/nick/swarm/devel/setup.bash

echo "Launching application, please wait!"
roslaunch fydp_mapping tb_dock_commands.launch command:=u
