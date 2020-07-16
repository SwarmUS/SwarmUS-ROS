#include "ros/ros.h"
#include "std_msgs/String.h"
#include "swarmus_ros_simulation/Communication_msg.h"

class InterCommunication
{
public:
    InterCommunication(std::string new_robot_name);
    ~InterCommunication();

    void publish(const swarmus_ros_simulation::Communication_msg& msg);

private:
    void communicationCallback(const std_msgs::String::ConstPtr& msg);

    std::string robot_name;

    // ROS NodeHandle
    ros::NodeHandle n;

    // ROS Topic Publishers
    ros::Publisher publisher;    

    ros::Subscriber subscriber;            
};