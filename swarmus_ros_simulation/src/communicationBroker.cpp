#include "swarmus_ros_simulation/communicationBroker.h"

CommunicationBroker::CommunicationBroker() {
  ROS_INFO("Communication Broker initialization");

  for (std::string& robot_name : Simulation::GetRobotList()) {
      publishersMap.emplace(robot_name, n.advertise<std_msgs::String>("CommunicationBroker/" + robot_name, 1000));
      subscribersList.push_back(n.subscribe("/" + robot_name + "/communication", 1000, &CommunicationBroker::communicationCallback, this));
  }
}

CommunicationBroker::~CommunicationBroker() {}

void CommunicationBroker::publishMsg(std::string robot_name, ros::Publisher pub, const swarmus_ros_simulation::Communication_msg& msg) {
  ROS_INFO("Publishing message: [%s] to [%s].", msg.message.c_str(), robot_name.c_str());
  std_msgs::String publishedMsg;
  publishedMsg.data = msg.message;
  pub.publish(publishedMsg);
}

void CommunicationBroker::communicationCallback(const swarmus_ros_simulation::Communication_msg& msg) {
    if (msg.target_robot == Simulation::Communication::AllRobots)
    {
      for (auto const& robotPublisher : publishersMap)              
        CommunicationBroker::publishMsg(robotPublisher.first, robotPublisher.second, msg);      
    }
    else if (msg.target_robot == Simulation::Communication::AllRobotsExceptSelf)
    {
      for (auto const& robotPublisher : publishersMap)
      {
        if (robotPublisher.first != msg.source_robot)                
          CommunicationBroker::publishMsg(robotPublisher.first, robotPublisher.second, msg);        
      }      
    }
    else {
      auto pubIter = publishersMap.find(msg.target_robot);
      if (pubIter != publishersMap.end()) 
        CommunicationBroker::publishMsg(msg.target_robot, pubIter->second, msg);      
    }
    
}

int main(int argc, char** argv){
  ros::init(argc, argv, "communication_broker");
  
  CommunicationBroker communicationBroker;
  ros::spin();
  
  return 0;
};