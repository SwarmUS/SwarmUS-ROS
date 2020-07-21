#include <map>

#include "ros/ros.h"
#include "simulationUtility.hpp"
#include "std_msgs/String.h"
#include "swarmus_ros_simulation/Communication_msg.h"

// Publie des messages
class CommunicationBroker
{
public:
  CommunicationBroker();

private:
  static void publishMsg(std::string robot_name, ros::Publisher pub,
                         const swarmus_ros_simulation::Communication_msg& msg);

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