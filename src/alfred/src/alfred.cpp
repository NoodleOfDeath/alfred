#include "ros/ros.h"
#include "alfred/CreateCommand.h"
#include <geometry_msgs/Twist.h>
#include <sstream>

#define _USE_MATH_DEFINES // for C++
#include <cmath>

#include "wrappers/create_driver.hpp"

CreateDriverROSWrapper create_driver;

void commandCallback(const alfred::CreateCommand::ConstPtr &msg)
{
    int command = msg->command;
    float velocity = msg->velocity;
    // ROS_INFO("Message Recieved: [%d, %.2f]", command, velocity);
    switch (command)
    {
    case alfred::CreateCommand::CMD_DRIVE:
        create_driver.drive((double)velocity);
        break;
    case alfred::CreateCommand::CMD_ROTATE:
        create_driver.rotate((double)velocity);
        break;
    }
}

/**
 * This is the main node that accesses all hardware drivers connected to ALFRED.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "alfred");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("batcomputer", 1000, commandCallback);
    create_driver = CreateDriverROSWrapper(node);
    ROS_INFO("CreateDriver Online");
    ros::spin();
    return 0;
}
