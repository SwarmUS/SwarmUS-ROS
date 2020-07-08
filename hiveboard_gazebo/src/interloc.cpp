#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>

#include <sstream>

float pos_x = 0.0;
float pos_y = 0.0;

void move(int delta_x, int delta_y) {
  pos_x += delta_x;
  pos_y += delta_y;
}

float getDistanceFrom(int x, int y) {
  return sqrt(pow((pos_x - x), 2) + pow((pos_y - y), 2));
}

float getAnglefrom(int x, int y) {
  // calculer un angle depuis un point
  return 90;
}

std::stringstream buildInterloc(float distance, float angle) {
  std::stringstream ss;
  ss << "[Robot 0] relative distance : " << distance << ", angle : " << angle;
  return ss;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interloc");
  ros::NodeHandle n;
  ros::Publisher interloc_pub = n.advertise<std_msgs::String>("interloc", 1000);
  ros::Rate loop_rate(10); // Probablement qu'on pourrait changer la boucle pour des evenements declenches par le board.
  
  while (ros::ok())
  {
    float distance = getDistanceFrom(0,0);
    float angle = getAnglefrom(0,0);

    std_msgs::String msg;
    msg.data = buildInterloc(distance, angle).str();

    ROS_INFO("%s", msg.data.c_str());

    interloc_pub.publish(msg); 

    // Loop and stuff
    move(1, 1);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
