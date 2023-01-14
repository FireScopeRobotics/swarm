# README

## How to run multi robot mapping demo:

1. Run roscore : `roscore`

2. Open the hospital multiple robot scene demo in Isaac Sim, the most recent Isaac Sim release is 2022.2.0. In Isaac Sim open a new stage and go to `omniverse://localhost/NVIDIA/Assets/Isaac/2022.2.0/Isaac/Samples/ROS/Scenario/multiple_robot_carter_hospital_navigation.usd`

3. Navigate to `swarm/src/fydp_mapping/launch/`
4. Source workspace. Run `roslaunch multiple_robot_carter_navigation.launch`

5. Source workspace. Run `roslaunch isaac_gmapping_test.launch`
6. Source workspace. Run `roslaunch isaac_merge_multiple_map_test.launch`

7. To control carter1 robot run: ` ROS_NAMESPACE=carter1 rosrun teleop_twist_keyboard teleop_twist_keyboard.py`. For controlling carter2 and carter3, use the same command but the ROS_NAMESPACE environment variable should be changed accordingly. :D
