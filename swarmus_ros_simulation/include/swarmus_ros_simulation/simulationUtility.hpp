#include <XmlRpcValue.h>

#include "ros/ros.h"

namespace Simulation
{
namespace Communication
{
const static std::string AllRobots = "allRobots";
const static std::string AllRobotsExceptSelf = "allRobotsExceptSelf";
}  // namespace Communication

static std::vector<std::string> GetRobotList()
{
  ros::NodeHandle nh;

  std::vector<std::string> robot_list;

  XmlRpc::XmlRpcValue config_robot_list;
  if (!nh.getParam("/robot_list", config_robot_list))
  {
    ROS_ERROR("No robot_list was found");
  }

  for (int i = 0; i < config_robot_list.size(); i++)
    robot_list.push_back(config_robot_list[i]);

  return robot_list;
}

static std::string GetParamRobotName()
{
  std::string robot_name;

  if (!ros::param::get("~robot_name", robot_name))  // The ~ is used to get param declared inside the <node></node> tags
  {
    ROS_INFO("No param name was given. pioneer_0 will be used instead");
    robot_name = "pioneer_0";
  }

  return robot_name;
}

struct Angle
{
  float inRadians;
  float inDegrees;
  Angle(float val, bool isRad)
  {
    if (isRad)
    {
      inRadians = val;
      inDegrees = val * 180 / M_PI;
    }
    else
    {
      inDegrees = val;
      inRadians = M_PI / (val * 180);
    }
  }
};

}  // namespace Simulation
