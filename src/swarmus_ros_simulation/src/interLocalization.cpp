#include "swarmus_ros_simulation/interLocalization.h"

/*
Class function implementations
*/
InterLocalization::InterLocalization() {
  robot_name = Simulation::GetParamRobotName();
  interloc_pub = node_handle.advertise<swarmus_ros_simulation::InterLocalization_grid>("interlocalization_grid", 1000);
  polygon_pub = node_handle.advertise<geometry_msgs::PolygonStamped>("PolygonStamped", 1000);

  ROS_INFO("HiveBoard initialization of: %s", robot_name.c_str());
}

float InterLocalization::getDistanceFrom(float x, float y) {
  return sqrt(pow(x, 2) + pow(y, 2));
}

float InterLocalization::getAnglefrom(float x, float y) {
  return atan2(y,x)*180.0/M_PI;
}


void InterLocalization::publish(swarmus_ros_simulation::InterLocalization_grid grid) {
  interloc_pub.publish(grid); 
  polygon_pub.publish(GeneratePolyMsg(grid));
}

const std::string InterLocalization::getRobotName() {
  return robot_name;
}
geometry_msgs::PolygonStamped InterLocalization::GeneratePolyMsg(swarmus_ros_simulation::InterLocalization_grid grid) {
  geometry_msgs::Polygon poly;
  poly.points.reserve(2 * grid.otherRobotsListSize+1);
  geometry_msgs::Point32 point;
  point.x = 0.0;
  point.y = 0.0;
  point.z = 0.0;
  poly.points.push_back(point);
  
  for(int i = 0; i < grid.otherRobotsListSize; i++) {
    swarmus_ros_simulation::InterLocalization interLocalization = grid.otherRobots[i];
    Simulation::Angle angle(interLocalization.angle, false);
    point.x = interLocalization.distance * cos(angle.inRadians);
    point.y = interLocalization.distance * sin(angle.inRadians);
    poly.points.push_back(point);

    // Add a point in polygon to return to hiveboard
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
    poly.points.push_back(point);
  }


  geometry_msgs::PolygonStamped msg;
  msg.polygon = poly;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = robot_name + "/hiveboard";
  return msg;
}

/*
ROS main
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "InterLocalization");
    
  InterLocalization interloc;

  std::string reference = interloc.getRobotName() + HIVEBOARD_LINK;
  swarmus_ros_simulation::InterLocalization_grid grid;
  grid.source_robot = reference;
  
  tf::TransformListener listener;

  ros::Rate loop_rate(10); // Probablement qu'on pourrait changer la boucle pour des evenements declenches par le board.

  while (ros::ok())
  {
    int count = 0;
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
        
      swarmus_ros_simulation::InterLocalization m;
      m.target_robot = target;
      m.distance = interloc.getDistanceFrom(transform.getOrigin().x(), transform.getOrigin().y());
      m.angle = interloc.getAnglefrom(transform.getOrigin().x(), transform.getOrigin().y());
      Simulation::Angle rotation(transform.getRotation().getAngle(), true);
      m.rotation = rotation.inDegrees;
      grid.otherRobots.push_back(m);
      count++;
    }

    grid.otherRobotsListSize = count;
    interloc.publish(grid);
    grid.otherRobots.clear();
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
