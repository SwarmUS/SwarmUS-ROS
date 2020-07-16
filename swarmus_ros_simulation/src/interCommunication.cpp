#include "swarmus_ros_simulation/interCommunication.h"

InterCommunication::InterCommunication(std::string new_robot_name) {
  ROS_INFO("HiveBoard communication initialization");

  robot_name = new_robot_name;

  publisher = n.advertise<swarmus_ros_simulation::Communication_msg>("communication", 1000);
  subscriber = n.subscribe("/CommunicationBroker/" + new_robot_name, 1000, &InterCommunication::communicationCallback, this);
}

InterCommunication::~InterCommunication() {}

void InterCommunication::publish(const swarmus_ros_simulation::Communication_msg& msg) {
  publisher.publish(msg);
}

void InterCommunication::communicationCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("[%s] InterCommunication heard: [%s]", robot_name.c_str(), msg->data.c_str());
}

int main(int argc, char** argv){
  ros::init(argc, argv, "hiveboard_interCommunications");

  std::string robot_name;
  if (!ros::param::get("~robot_name",robot_name)) // The ~ is used to get param declared inside the <node></node> tags
  {
    ROS_INFO("No param name was given. pioneer_0 will be used instead");
    robot_name = "pioneer_0";
  }

  InterCommunication interCommunication(robot_name);

  // Send message each second
  ros::Rate loop_rate(1);

  // For current implementation, pioneer_0 only will send a message to pioneer_1.
  while(ros::ok()) {
    swarmus_ros_simulation::Communication_msg msg;
    if (robot_name == "pioneer_0") {
      msg.source_robot = robot_name;
      msg.target_robot = Simulation::Communication::AllRobotsExceptSelf;
      msg.message = robot_name + " say hello.";

      //ROS_INFO("%s", msg.data.c_str());

      interCommunication.publish(msg);
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
};