#include "hiveboard_gazebo/interloc.h"

/*
Class function implementations
*/

Interloc::Interloc() {
  interloc_pub = n.advertise<std_msgs::String>("interloc", 1000);
}

float Interloc::getDistanceFrom(int x, int y) {
  return sqrt(pow((pos_x - x), 2) + pow((pos_y - y), 2));
}

float Interloc::getAnglefrom(int x, int y) {
  // calculer un angle depuis un point
  return 90;
}


void Interloc::publish(std_msgs::String msg) {
    interloc_pub.publish(msg); 
}

void Interloc:: move(int delta_x, int delta_y) {
  pos_x += delta_x;
  pos_y += delta_y;
}

/*
Helpers
*/
std::stringstream buildInterloc(float distance, float angle) {
  std::stringstream ss;
  ss << "[Robot 0] relative distance : " << distance << ", angle : " << angle;
  return ss;
}

/*
ROS main
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "interloc");
  Interloc interloc;

  ros::Rate loop_rate(10); // Probablement qu'on pourrait changer la boucle pour des evenements declenches par le board.
  
  while (ros::ok())
  {
    float distance = interloc.getDistanceFrom(0,0);
    float angle = interloc.getAnglefrom(0,0);

    std_msgs::String msg;
    msg.data = buildInterloc(distance, angle).str();

    ROS_INFO("%s", msg.data.c_str());

    // Loop and stuff
    interloc.move(1, 1);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
