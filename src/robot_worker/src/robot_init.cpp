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