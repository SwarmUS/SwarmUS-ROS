#include "ros/ros.h"
#include "std_msgs/String.h"

class InterCommunication
{
public:
    InterCommunication();

    ~InterCommunication();

private:
    // ROS NodeHandle
    ros::NodeHandle n;

    // ROS Topic Publishers
    ros::Publisher interloc_pub;
};