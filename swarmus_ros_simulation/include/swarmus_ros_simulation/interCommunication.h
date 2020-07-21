#include "ros/ros.h"
#include "std_msgs/String.h"
#include "swarmus_ros_simulation/Communication.h"
#include "swarmus_ros_simulation/simulationUtility.hpp"

class InterCommunication
{
public:
    InterCommunication();    

    void publish(const swarmus_ros_simulation::Communication& msg);
    const std::string getRobotName();

private:
    void communicationCallback(const std_msgs::String::ConstPtr& msg);

    std::string robot_name;

    // ROS NodeHandle
    ros::NodeHandle node_handle;

    // ROS Topic Publishers
    ros::Publisher publisher;    

    ros::Subscriber subscriber;            
};