#include "ros/ros.h"
#include "std_msgs/String.h"
#include "swarmus_ros_simulation/InterLocalization.h"
#include "swarmus_ros_simulation/InterLocalization_grid.h"
#include "swarmus_ros_simulation/simulationUtility.hpp"
#include <math.h>
#include <tf/transform_listener.h>
#include <string>
#include <XmlRpcValue.h> 
#include <sstream>

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

        std::string robot_name;
};