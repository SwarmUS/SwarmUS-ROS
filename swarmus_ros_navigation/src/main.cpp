#include <ros/ros.h>
#include "Navigation.hpp" 

int main(int argc, char** argv)
{
  //  Initialize rosbuzz node
  ros::init(argc, argv, "navigation");
  static ros::NodeHandle nodeHandle("~");
  
  // Initialize and start node execution
  Navigation navigation(&nodeHandle);
  navigation.execute();


  return 0;
}