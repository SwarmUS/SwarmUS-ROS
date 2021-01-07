#ifndef _INTERLOCALIZATION_H
#define _INTERLOCALIZATION_H

#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "swarmus_ros_simulation/InterLocalization.h"
#include "swarmus_ros_simulation/InterLocalization_grid.h"
#include "swarmus_ros_simulation/simulationUtility.hpp"
#include <XmlRpcValue.h>
#include <math.h>
#include <sstream>
#include <string>
#include <tf/transform_listener.h>

static const std::string gs_hiveBoardLink = "/hiveboard";

class InterLocalization {
  public:
    InterLocalization();
    float getDistanceFrom(float x, float y);
    float getAnglefrom(float x, float y);
    void publish(swarmus_ros_simulation::InterLocalization_grid grid);
    std::string getRobotName();

  private:
    // ROS NodeHandle
    ros::NodeHandle m_nodeHandle;

    // ROS Topic Publishers
    ros::Publisher m_interlocPub;
    ros::Publisher m_polygonPub;

    std::string m_robotName;

    geometry_msgs::PolygonStamped generatePolyMsg(
        swarmus_ros_simulation::InterLocalization_grid grid);
};

#endif // _INTERLOCALIZATION_H
