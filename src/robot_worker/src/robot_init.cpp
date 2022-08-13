#include "robot_worker/robot.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "robot");
    ros::NodeHandle n("~");

    int robot_idx = 0;
    if (!n.hasParam("robot_idx"))
    {
      ROS_INFO("cannot find robot_idx param'");
    }
    else{
      n.getParam("robot_idx", robot_idx);
      Robot robot(n, robot_idx);

      ROS_INFO("ROBOT NODE SETUP COMPLETE FOR ROBOT: %d", robot_idx);

    }


    if (ros::ok()){
      ros::spin();
    }

  return 0;
}

// uint8 function_type
// int8 num_robots
// float32[2] leader_des
// float32[2] v
// float32 l


// float32 drone_vel


// std_msgs/Float32MultiArray obstacles
//   std_msgs/MultiArrayLayout layout
//     std_msgs/MultiArrayDimension[] dim
//       string label
//       uint32 size
//       uint32 stride
//     uint32 data_offset
//   float32[] data


// float32 influence_radius
// std_msgs/Float32MultiArray P
//   std_msgs/MultiArrayLayout layout
//     std_msgs/MultiArrayDimension[] dim
//       string label
//       uint32 size
//       uint32 stride
//     uint32 data_offset
//   float32[] data
// float32 ViconRate


// ---
// std_msgs/Float32MultiArray destinations
//   std_msgs/MultiArrayLayout layout
//     std_msgs/MultiArrayDimension[] dim
//       string label
//       uint32 size
//       uint32 stride
//     uint32 data_offset
//   float32[] data
// float32[2] sp
// float32[2] nxt_pt

