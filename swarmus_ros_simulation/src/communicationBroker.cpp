#include "swarmus_ros_simulation/communicationBroker.h"

CommunicationBroker::CommunicationBroker() {
  ROS_INFO("Communication Broker initialization");

  for (std::string& robot_name : Simulation::GetRobotList()) {
      publishersMap.emplace(robot_name, n.advertise<std_msgs::String>("CommunicationBroker/" + robot_name, 1000));
      subscribersList.push_back(n.subscribe("/" + robot_name + "/communication", 1000, &CommunicationBroker::communicationCallback, this));
  }
}

CommunicationBroker::~CommunicationBroker() {}

void CommunicationBroker::communicationCallback(const swarmus_ros_simulation::Communication_msg& msg) {
    auto pubIter = publishersMap.find(msg.target_robot);
    if (pubIter != publishersMap.end()) {
        ROS_INFO("Publishing message: [%s] to [%s].", msg.message.c_str(), msg.target_robot.c_str());
        std_msgs::String publishedMsg;
        publishedMsg.data = msg.message;
        pubIter->second.publish(publishedMsg);
    }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "communication_broker");
  
  CommunicationBroker communicationBroker;
  ros::spin();
  
  return 0;
};