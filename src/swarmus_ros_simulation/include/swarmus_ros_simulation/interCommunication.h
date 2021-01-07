#ifndef _INTERCOMMUNICATION_H
#define _INTERCOMMUNICATION_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "swarmus_ros_simulation/Communication.h"
#include "swarmus_ros_simulation/simulationUtility.hpp"

class InterCommunication {
  public:
    InterCommunication();

    void publish(const swarmus_ros_simulation::Communication& msg);
    std::string getRobotName();

  private:
    void communicationCallback(const std_msgs::String::ConstPtr& msg);

    std::string m_robotName;

    // ROS NodeHandle
    ros::NodeHandle m_nodeHandle;

    // ROS Topic publishers
    ros::Publisher m_publisher;

    ros::Subscriber m_subscriber;
};

#endif // #ifndef _INTERCOMMUNICATION_H
