#include "swarmus_ros_simulation/interLocalization.h"

/*
Class function implementations
*/
InterLocalization::InterLocalization(std::string new_robot_name) {
  robot_name = new_robot_name;
  interloc_pub = n.advertise<swarmus_ros_simulation::InterLocalization_grid_msg>("interlocalization_grid", 1000);

  ROS_INFO("HiveBoard initialization of: %s", robot_name.c_str());
}

InterLocalization::~InterLocalization() {}

float InterLocalization::getDistanceFrom(float x, float y) {
  return sqrt(pow(x, 2) + pow(y, 2));
}

float InterLocalization::getAnglefrom(float x, float y) {
  return atan2(y,x)*180.0/M_PI;
}


void InterLocalization::publish(swarmus_ros_simulation::InterLocalization_grid_msg grid) {
    interloc_pub.publish(grid); 
}

/*
Helpers
*/
std::string getParamRobotName() {
  std::string robot_name;

  if (!ros::param::get("~robot_name",robot_name)) // The ~ is used to get param declared inside the <node></node> tags
  {
    ROS_INFO("No param name was given. pioneer_0 will be used instead");
    robot_name = "pioneer_0";
  }

  return robot_name;
}

float toDegrees(float rad) {
  return (rad * 180/M_PI);
}

/*
ROS main
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "InterLocalization");
  ros::NodeHandle n;
  
  std::string robot_name = getParamRobotName();
  InterLocalization interloc(robot_name);

  std::string reference = interloc.robot_name + HIVEBOARD_LINK;
  swarmus_ros_simulation::InterLocalization_grid_msg grid;
  grid.source_robot = reference;
  
  tf::TransformListener listener;

  int count = 0;
  ros::Rate loop_rate(10); // Probablement qu'on pourrait changer la boucle pour des evenements declenches par le board.

  while (ros::ok())
  {
    for(std::string& robot_name : Simulation::GetRobotList())
    {
      if (robot_name == interloc.robot_name)
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
        
      swarmus_ros_simulation::InterLocalization_msg i;
      i.target_robot = target;
      i.distance = interloc.getDistanceFrom(transform.getOrigin().x(), transform.getOrigin().y());
      i.angle = interloc.getAnglefrom(transform.getOrigin().x(), transform.getOrigin().y());
      i.rotation = toDegrees(transform.getRotation().getAngle());
      grid.otherRobots.push_back(i);
    }

    interloc.publish(grid);
    grid.otherRobots.clear();
    
    ++count;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
