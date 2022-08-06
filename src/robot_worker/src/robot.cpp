#include "robot_worker/robot.h"

Robot::Robot(ros::NodeHandle n, int robot_idx){

    //self parameters
    m_robot_idx = robot_idx;
    m_is_leader = false;
    m_robot_name = "robot_" + std::to_string(robot_idx);


    //pub
    motor_control_pub = n.advertise<std_msgs::Float32MultiArray>("motorControl", 10);

    //sub
    // odom_sub = n.subscribe("odom", 2, Robot::odomCallback);
    odom_sub = n.subscribe("odom", 2, &Robot::odomCallback, this);
}

void Robot::publishMotorControl(float speed[2])
{
  std_msgs::Float32MultiArray msg;
  msg.data = {speed[0], speed[2]};
  motor_control_pub.publish(msg);
  
}

void Robot::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("%s Got odom messge with id %d", m_robot_name.c_str(),  msg->header.seq);
}