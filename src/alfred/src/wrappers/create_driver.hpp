#include <ros/ros.h>

/**
 * @brief ROS wrapper class for publishing messages to the iRobot Create 2 machine.
 *
 */
class CreateDriverROSWrapper
{
public:
    CreateDriverROSWrapper();
    CreateDriverROSWrapper(ros::NodeHandle node);
    void drive(double velocity);
    void rotate(double velocity);

private:
    ros::Publisher pub_cmd_vel;
};