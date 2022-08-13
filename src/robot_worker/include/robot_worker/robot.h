#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <sstream>
#include <robot_worker/convoy_functions.h>

class Robot{
private:
    //variables
    ros::Publisher motor_control_pub;
    ros::Publisher robot_comm_pub; //TODO: this is for communication between robots, all robots will subscribe to this so that this message will be broadcast
    ros::Subscriber odom_sub;
    ros::Subscriber robot_comm_sub;

    ros::ServiceClient m_client;

    bool m_is_leader;
    int m_robot_idx;
    std::string m_robot_name;
public:
    //api
    Robot(ros::NodeHandle n, int robot_idx);
private:
    void publishMotorControl(float speed[2]);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

//callbacks
// void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);




// extern Robot robot;
