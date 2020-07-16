#include <XmlRpcValue.h> 
#include "ros/ros.h"

namespace Simulation {

    namespace Communication {
        std::string AllRobots = "allRobots";
        std::string AllRobotsExceptSelf = "allRobotsExceptSelf";
    }

    static std::vector<std::string> GetRobotList() {
            ros::NodeHandle nh;

            std::vector<std::string> robot_list;

            XmlRpc::XmlRpcValue v;
            if (!nh.getParam("/robot_list", v))
            {
                ROS_ERROR("No robot_list was found");
            }

            for(int i =0; i < v.size(); i++)            
                robot_list.push_back(v[i]);

            return robot_list;    
    }

}
