#include "ros/ros.h"
#include "std_msgs/String.h"
#include "swarmus_ros_simulation/Communication_msg.h"
#include "simulationUtility.hpp"
#include <map>

// Publie des messages
class CommunicationBroker
{
public:
    CommunicationBroker();
    ~CommunicationBroker();

private:
    
    // Contains all robot associated with their publisher
    std::map<std::string, ros::Publisher> publishersMap;
    // Contains all robot's subscriber
    std::vector<ros::Subscriber> subscribersList; 

    // Called whenever a subscriber gets a message
    void communicationCallback(const swarmus_ros_simulation::Communication_msg& msg);  

    // ROS NodeHandle
    ros::NodeHandle n;

    // ROS Topic Publisher
    ros::Publisher publisher;    

    // ROS Topic subscriber
    ros::Subscriber subscriber;
};