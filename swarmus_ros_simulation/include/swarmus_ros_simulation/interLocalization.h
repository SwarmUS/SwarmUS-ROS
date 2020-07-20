#include "ros/ros.h"
#include "std_msgs/String.h"
#include "swarmus_ros_simulation/InterLocalization_msg.h"
#include "swarmus_ros_simulation/InterLocalization_grid_msg.h"
#include "swarmus_ros_simulation/simulationUtility.hpp"
#include <math.h>
#include <tf/transform_listener.h>
#include <string>
#include <XmlRpcValue.h> 
#include <sstream>

// TODO change std::String to std_msgs::String
const std::string HIVEBOARD_LINK = "/hiveboard";

class InterLocalization {
    public:
        InterLocalization(std::string new_robot_name);
        ~InterLocalization();
        float getDistanceFrom(float x, float y);
        float getAnglefrom(float x, float y);
        void publish(swarmus_ros_simulation::InterLocalization_grid_msg grid);
        void move(int delta_x, int delta_y);


        std::string robot_name;

    private:
        // ROS NodeHandle
        ros::NodeHandle n;

        // ROS Topic Publishers
        ros::Publisher interloc_pub;
};