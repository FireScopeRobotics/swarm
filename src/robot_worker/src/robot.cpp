#include "robot_worker/robot.h"

Robot::Robot(ros::NodeHandle n, int robot_idx){

    //self parameters
    m_robot_idx = robot_idx;
    m_is_leader = false;
    m_robot_name = "robot_" + std::to_string(robot_idx);

    m_client = n.serviceClient<robot_worker::convoy_functions>("convoy_functions");

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

std_msgs::Float32MultiArray Robot::formation(int num_robots, boost::array<float,2> leader_des, boost::array<float,2> v, float l)
{
  robot_worker::convoy_functions srv;

  int func_type = 0;
  srv.request.function_type = func_type;
  srv.request.num_robots = num_robots;
  srv.request.leader_des = leader_des;
  srv.request.v = v;
  srv.request.l = l;

  if (m_client.call(srv))
  {
    ROS_INFO("convoy_functions/formation called");
    return srv.response.destinations;
  }
  else
  {
    ROS_ERROR("convoy_functions/formation failed");
    throw 1;
  }
}

boost::array<float, 2> Robot::nextStepFromSingleWPT(float drone_vel, float vicon_rate, std_msgs::Float32MultiArray P)
{
  robot_worker::convoy_functions srv;

  int func_type = 1;
  srv.request.function_type = func_type;
  srv.request.drone_vel = drone_vel;
  srv.request.ViconRate = vicon_rate;
  srv.request.P = P;

  if (m_client.call(srv))
  {
    ROS_INFO("convoy_functions/nextStepFromSingleWPT called");
    return srv.response.nxt_pt;
  }
  else
  {
    ROS_ERROR("convoy_functions/nextStepFromSingleWPT failed");
    throw 1;
  }
}

boost::array<float, 2> Robot::local_planner(float drone_vel, float influence_radius, std_msgs::Float32MultiArray obstacles)
{
  robot_worker::convoy_functions srv;

  int func_type = 2;
  srv.request.function_type = func_type;
  srv.request.drone_vel = drone_vel;
  srv.request.influence_radius = influence_radius;
  srv.request.obstacles = obstacles;

  if (m_client.call(srv))
  {
    ROS_INFO("convoy_functions/local_planner called");
    return srv.response.sp;
  }
  else
  {
    ROS_ERROR("convoy_functions/local_planner failed");
    throw 1;
  }
}