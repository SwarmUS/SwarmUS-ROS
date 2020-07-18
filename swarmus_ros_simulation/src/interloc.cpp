#include "swarmus_ros_simulation/interloc.h"

/*
Class function implementations
*/

Interloc::Interloc(std::string new_robot_name, float x, float y) {
  robot_name = new_robot_name;
  interloc_pub = n.advertise<swarmus_ros_simulation::Interloc_grid>("interloc_grid", 1000);
  pos_x = x;
  pos_y = y;

  ROS_INFO("HiveBoard initialization of: %s", robot_name.c_str());
}

Interloc::~Interloc() {}

float Interloc::getDistanceFrom(float x, float y) {
  return sqrt(pow((pos_x - x), 2) + pow((pos_y - y), 2));
}

float Interloc::getAnglefrom(float x, float y) {
  // calculer un angle depuis un point
  return atan2(y,x)*180.0/M_PI;
}


void Interloc::publish(swarmus_ros_simulation::Interloc_grid grid) {
    interloc_pub.publish(grid); 
}

/*
Helpers
*/
std::stringstream buildInterloc(float distance, float angle) {
  std::stringstream ss;
  ss << "[Robot 0] relative distance : " << distance << ", angle : " << angle;
  return ss;
}

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
  Interloc interloc(robot_name, pos_x, pos_y);

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
        // TODO update position of self
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
        
      float dist = interloc.getDistanceFrom(transform.getOrigin().x(), transform.getOrigin().y());
      float angle = interloc.getAnglefrom(transform.getOrigin().x(), transform.getOrigin().y());
      ROS_INFO("Distance: %f, Angle: %f, iteration: %d", dist, angle, count); // Only for debug

      // Create interloc msg
      swarmus_ros_simulation::Interloc i;
      i.target_robot = target;
      i.distance = dist;
      i.angle = angle;

      // append to interloc grid
      grid.otherRobots.push_back(i);
    }

    // Publish interloc grid
    interloc.publish(grid);
    grid.otherRobots.clear();
    
    // ROS loop and stuff
    ++count;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
