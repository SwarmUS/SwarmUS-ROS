#include "swarmus_ros_simulation/interLocalization.h"

/*
Class function implementations
*/
InterLocalization::InterLocalization() {
  robot_name = Simulation::GetParamRobotName();
  interloc_pub = n.advertise<swarmus_ros_simulation::InterLocalization_grid_msg>("interlocalization_grid", 1000);

  ROS_INFO("HiveBoard initialization of: %s", robot_name.c_str());
}

float InterLocalization::getDistanceFrom(float x, float y) {
  return sqrt(pow(x, 2) + pow(y, 2));
}

float InterLocalization::getAnglefrom(float x, float y) {
  return atan2(y,x)*180.0/M_PI;
}


void InterLocalization::publish(swarmus_ros_simulation::InterLocalization_grid_msg grid) {
  interloc_pub.publish(grid); 
}

const std::string InterLocalization::getRobotName() {
  return robot_name;
}

/*
ROS main
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "InterLocalization");
    
  InterLocalization interloc;

  std::string reference = interloc.getRobotName() + HIVEBOARD_LINK;
  swarmus_ros_simulation::InterLocalization_grid_msg grid;
  grid.source_robot = reference;
  
  tf::TransformListener listener;

  int count = 0;
  ros::Rate loop_rate(10); // Probablement qu'on pourrait changer la boucle pour des evenements declenches par le board.

  while (ros::ok())
  {
    for(std::string& robot_name : Simulation::GetRobotList())
    {
      if (robot_name == interloc.getRobotName())
      {
        continue;
      }

      std::string target = robot_name + HIVEBOARD_LINK;

      tf::StampedTransform transform;
      try{
        listener.lookupTransform(reference, target,  
                              ros::Time(0), transform);
      }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
        
      swarmus_ros_simulation::InterLocalization_msg m;
      m.target_robot = target;
      m.distance = interloc.getDistanceFrom(transform.getOrigin().x(), transform.getOrigin().y());
      m.angle = interloc.getAnglefrom(transform.getOrigin().x(), transform.getOrigin().y());
      Simulation::Angle rotation(transform.getRotation().getAngle(), true);
      m.rotation = rotation.inDegrees;
      grid.otherRobots.push_back(m);
    }

    interloc.publish(grid);
    grid.otherRobots.clear();
    
    ++count;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
