# swarm
Monorepo for the swarm bot design project.


# Initial Setup

Install Isaac sim. See [install instructions](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_basic.html)
Also make sure to setup cache and nucleus as instructed.

## Building the workspace
Install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu). 

Install [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).

If not already done so, [install rodep](http://wiki.ros.org/rosdep). Follow steps up to and including 2.2.

Install ROS Navigation stack: `sudo apt-get install ros-$ROS_DISTRO-navigation`.

Go to the root of this workspace. Run the rosdep command to install any missing dependencies: `rosdep install --from-paths src --ignore-src -r -y`.

Build the workspace: `catkin build`
Source the workspace: `source devel/setup.bash`


# Fydp_mapping

Fydp mapping package allows for multi robot mapping. See readme inside fydp_mapping folder for instructions on running demo.


