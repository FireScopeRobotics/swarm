#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <sstream>
class Robot{
private:
    //variables
    ros::Publisher motor_control_pub;
    ros::Subscriber odom_sub;

    std::string name;
public:
    //api
    Robot(ros::NodeHandle n); 
private:
    void publishMotorControl(float speed[2]);
    friend void odomCallback(const nav_msgs::Odometry::ConstPtr&);
};

//callbacks
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);







extern Robot robot;