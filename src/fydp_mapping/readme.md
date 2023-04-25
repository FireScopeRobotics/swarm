# README

## How to run multi robot mapping demo with exploration:

1. Run roscore : `roscore`

2. Open the hospital multiple robot scene demo in Isaac Sim, the most recent Isaac Sim release is 2022.2.0. In Isaac Sim open a new stage and go to `omniverse://localhost/Users/ayushg/exploration_multi_robot_hospital.usd`

3. Navigate to `swarm/src/fydp_mapping/launch/`
4. Source workspace. Run `roslaunch fydp_mapping multiple_robot_navigation.launch`
5. Source workspace. Run `roslaunch fydp_mapping topic_inject.launch`
6. Source workspace. Run `roslaunch fydp_mapping dock_commands.launch command:=u`
7. Source workspace. Run `roslaunch fydp_mapping isaac_gmapping_test.launch`
8. Source workspace. Run `roslaunch fydp_mapping isaac_merge_multiple_map_test.launch`
9. Source workspace. Run `roslaunch explore_lite explore_costmap_multi_robot_test.launch`

## How to run multi robot mapping demo (No Exploration):

1. Run roscore : `roscore`

2. Source workspace. Run `roslaunch fydp_mapping multiple_robot_navigation.launch`

3. Source workspace. Run `roslaunch isaac_gmapping_test.launch`

4. Source workspace. Run `roslaunch isaac_merge_multiple_map_test.launch`

5. To control carter1 robot run: ` ROS_NAMESPACE=carter1 rosrun teleop_twist_keyboard teleop_twist_keyboard.py`. For controlling carter2 and carter3, use the same command but the ROS_NAMESPACE environment variable should be changed accordingly. :D






FOR DEMO:

roslaunch fydp_mapping multiple_robot_navigation.launch

roslaunch fydp_mapping topic_inject.launch

roslaunch fydp_mapping merge_multiple_map.launch

roslaunch fydp_mapping dock_commands.launch command:=u

rosrun fydp_mapping map_listener.py
OR 
roslaunch fydp_mapping fydp_slam_toolbox_map.launch
(OR python3 ~/swarm/src/fydp_mapping/scripts/map_listener.py )

rosrun fydp_mapping explore_listener.py
OR
roslaunch explore_lite explore_slam_costmap_multi_robot.launch
(OR python3 ~/swarm/src/fydp_mapping/scripts/explore_listener.py )


rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=carter1/cmd_vel

OR for joystick:

rosrun joy joy_node

rosrun fydp_mapping turtlebot4_joystick.py