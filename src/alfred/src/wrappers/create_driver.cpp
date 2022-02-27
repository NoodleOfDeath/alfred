#include <ros/ros.h>
#include <memory>
#include <geometry_msgs/Twist.h>
#include "create_driver.hpp"

CreateDriverROSWrapper::CreateDriverROSWrapper()
{
}

CreateDriverROSWrapper::CreateDriverROSWrapper(ros::NodeHandle node)
{
    this->pub_cmd_vel = node.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
}

void CreateDriverROSWrapper::drive(double velocity)
{
    // Prevent the robot from going more than 0.5m per second foward or backward.
    if (velocity < -0.5 || velocity > 0.5)
        return;
    geometry_msgs::Twist msg;
    msg.linear.x = velocity;
    msg.angular.z = 0.0;
    this->pub_cmd_vel.publish(msg);
}

void CreateDriverROSWrapper::rotate(double velocity)
{
    // Prevent the robot from turning more than 4.25 radians per second clockwise or counter-clockwise.
    if (velocity < -4.25 || velocity > 4.25)
        return;
    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = velocity;
    this->pub_cmd_vel.publish(msg);
}