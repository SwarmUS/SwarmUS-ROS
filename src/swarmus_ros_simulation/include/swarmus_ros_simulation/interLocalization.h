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

const std::string HIVEBOARD_LINK = "/hiveboard";

class InterLocalization {
  public:
    InterLocalization();
    float getDistanceFrom(float x, float y);
    float getAnglefrom(float x, float y);
    void publish(swarmus_ros_simulation::InterLocalization_grid grid);
    const std::string getRobotName();

  private:
    // ROS NodeHandle
    ros::NodeHandle node_handle;

    // ROS Topic Publishers
    ros::Publisher interloc_pub;
    ros::Publisher polygon_pub;

    std::string robot_name;

    geometry_msgs::PolygonStamped GeneratePolyMsg(
        swarmus_ros_simulation::InterLocalization_grid grid);
};