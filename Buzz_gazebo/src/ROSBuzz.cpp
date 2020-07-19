
#include <ros/ros.h>
#include "BuzzBridge.hpp" 

int main(int argc, char** argv)
{
  //  Initialize rosbuzz node
  ros::init(argc, argv, "rosbuzz");
  static ros::NodeHandle nodeHandle("/rosbuzz_node");
  std::string t;
  
  // Initialize and start node execution
  BuzzBridge bridge(&nodeHandle);
  bridge.execute();


  return 0;
}