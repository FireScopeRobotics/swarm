#include "robot_worker/robot.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "robot");
    ros::NodeHandle n;

    Robot robot(n, 0);

    ROS_INFO("ROBOT NODE SETUP COMPLETE");

    if (ros::ok()){
        ros::spin();
    }

  return 0;
}