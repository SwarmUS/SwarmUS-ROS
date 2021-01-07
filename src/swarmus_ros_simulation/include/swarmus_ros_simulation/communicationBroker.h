#ifndef _COMMUNICATIONBROKER_H
#define _COMMUNICATIONBROKER_H

#include "ros/ros.h"
#include "simulationUtility.hpp"
#include "std_msgs/String.h"
#include "swarmus_ros_simulation/Communication.h"
#include <map>

// Publishes messages
class CommunicationBroker {
  public:
    CommunicationBroker();

  private:
    static void publishMsg(std::string robot_name,
                           ros::Publisher pub,
                           const swarmus_ros_simulation::Communication& msg);

    // Contains all robot associated with their publisher
    std::map<std::string, ros::Publisher> m_publishersMap;

    // Contains all robot's subscriber
    std::vector<ros::Subscriber> m_subscribersList;

    // Called whenever a subscriber gets a message
    void communicationCallback(const swarmus_ros_simulation::Communication& msg);

    // ROS NodeHandle
    ros::NodeHandle m_nodeHandle;

    // ROS Topic Publisher
    ros::Publisher m_publisher;

    // ROS Topic subscriber
    ros::Subscriber m_subscriber;
};

#endif // _COMMUNICATIONBROKER_H
