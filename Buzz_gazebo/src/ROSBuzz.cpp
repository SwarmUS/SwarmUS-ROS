
#include <ros/ros.h>

int main(int argc, char** argv)
{
  //  Initialize rosBuzz node
  ros::init(argc, argv, "rosBuzz");
  ros::NodeHandle nh_priv("~");
  ros::NodeHandle nh;
  

  return 0;
}