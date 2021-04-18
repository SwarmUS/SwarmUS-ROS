#ifndef _SIMULATIONUTILITY_HPP
#define _SIMULATIONUTILITY_HPP

#include "ros/ros.h"
#include <XmlRpcValue.h>

namespace Simulation {

    namespace Communication {

        const std::string allRobots = "allRobots";
        const std::string allRobotsExceptSelf = "allRobotsExceptSelf";
    } // namespace Communication

    std::vector<std::string> getRobotList() {
        ros::NodeHandle nh;

        std::vector<std::string> robot_list;

        XmlRpc::XmlRpcValue config_robot_list;
        if (!nh.getParam("/robot_list", config_robot_list)) {
            ROS_ERROR("No robot_list was found");
        }

        for (int i = 0; i < config_robot_list.size(); i++)
            robot_list.push_back(config_robot_list[i]);

        return robot_list;
    }

    std::string getParamRobotName() {
        std::string robot_name;

        if (!ros::param::get(
                "~robot_name",
                robot_name)) // The ~ is used to get param declared inside the <node></node> tags
        {
            ROS_INFO("No param name was given. pioneer_1 will be used instead");
            robot_name = "pioneer_1";
        }

        return robot_name;
    }

    struct Angle {
        float inRadians;
        float inDegrees;
        Angle(float val, bool isRad) {
            if (isRad) {
                inRadians = val;
                inDegrees = val * 180 / M_PI;
            } else {
                inDegrees = val;
                inRadians = val * M_PI / 180;
            }
        }
    };

} // namespace Simulation

#endif // _SIMULATIONUTILITY_HPP
