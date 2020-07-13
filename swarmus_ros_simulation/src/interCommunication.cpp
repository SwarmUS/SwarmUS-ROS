#include "swarmus_ros_simulation/interCommunication.h"

InterCommunication::InterCommunication() {

    interloc_pub = n.advertise<std_msgs::String>("interCommunication", 1000);

    ROS_INFO("HiveBoard communication initialization");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "hiveboard_interCommunications");

  ros::NodeHandle node;

  ros::spin();
  return 0;
};