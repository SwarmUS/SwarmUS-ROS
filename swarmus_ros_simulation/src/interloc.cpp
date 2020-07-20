#include "swarmus_ros_simulation/interloc.h"

/*
Class function implementations
*/
Interloc::Interloc(std::string new_robot_name) {
  robot_name = new_robot_name;
  interloc_pub = n.advertise<swarmus_ros_simulation::Interloc_grid>("interloc_grid", 1000);

  ROS_INFO("HiveBoard initialization of: %s", robot_name.c_str());
}

Interloc::~Interloc() {}

float Interloc::getDistanceFrom(float x, float y) {
  return sqrt(pow(x, 2) + pow(y, 2));
}

float Interloc::getAnglefrom(float x, float y) {
  // TODO calculer un angle en prenant en considerant l'orientation
  // de self
  return atan2(y,x)*180.0/M_PI;
}


void Interloc::publish(swarmus_ros_simulation::Interloc_grid grid) {
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

float getParamPosX() {
  float pos_x;

  if (!ros::param::get("~pos_x",pos_x)) // The ~ is used to get param declared inside the <node></node> tags
  {
    ROS_INFO("No param pos_x was given. 0.0 will be used instead");
    pos_x = 0.0;
  }

  return pos_x;
}

float getParamPosY() {
  float pos_y;

  if (!ros::param::get("~pos_y",pos_y)) // The ~ is used to get param declared inside the <node></node> tags
  {
    ROS_INFO("No param pos_y was given. 0.0 will be used instead");
    pos_y = 0.0;
  }

  return pos_y;
}

float toDegrees(float rad) {
  return (rad * 180/M_PI);
}

/*
ROS main
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "interloc");
  ros::NodeHandle n;
  
  std::string robot_name = getParamRobotName();
  float pos_x = getParamPosX();
  float pos_y = getParamPosY();
  Interloc interloc(robot_name);

  std::string reference = interloc.robot_name + HIVEBOARD_LINK;
  swarmus_ros_simulation::Interloc_grid grid;
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
        
      swarmus_ros_simulation::Interloc i;
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
