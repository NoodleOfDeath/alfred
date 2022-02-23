#include "ros/ros.h"
#include "alfred/Float32List.h"
#include <geometry_msgs/Twist.h>
#include <sstream>

#define _USE_MATH_DEFINES // for C++
#include <cmath>

#include "wrappers/create_driver.hpp"

CreateDriverROSWrapper create_driver;

void commandCallback(const alfred::Float32List::ConstPtr &msg)
{
    int command = (int)msg->data[0];
    float speed = msg->data[1];
    ROS_INFO("Message Recieved: [%d, %.2f]", command, speed);
    switch (command)
    {
    case 1:
        create_driver.drive((double)speed);
        break;
    case 2:
        create_driver.rotate((double)speed);
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
