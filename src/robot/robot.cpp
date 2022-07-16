#include "./robot.h"

Robot::Robot(ros::NodeHandle n){
    motor_control_pub = n.advertise<std_msgs::Float32MultiArray>("motorControl", 10);
    odom_sub = n.subscribe("odom", 2, odomCallback);
}

void Robot::publishMotorControl(float speed[2])
{
  std_msgs::Float32MultiArray msg;
  msg.data = {speed[0], speed[2]};
  motor_control_pub.publish(msg);
  
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("%s Got odom messge with id %d", robot.name,  msg->header.seq);
}