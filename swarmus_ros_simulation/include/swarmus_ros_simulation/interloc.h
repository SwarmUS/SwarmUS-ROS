#include "ros/ros.h"
#include "std_msgs/String.h"
#include "swarmus_ros_simulation/Interloc.h"
#include "swarmus_ros_simulation/Interloc_grid.h"
#include "swarmus_ros_simulation/simulationUtility.hpp"
#include <math.h>
#include <tf/transform_listener.h>
#include <string>
#include <XmlRpcValue.h> 
#include <sstream>

// TODO change std::String to std_msgs::String
const std::string HIVEBOARD_LINK = "/hiveboard";

class Interloc {
    public:
        Interloc(std::string new_robot_name);
        ~Interloc();
        float getDistanceFrom(float x, float y);
        float getAnglefrom(float x, float y);
        void publish(swarmus_ros_simulation::Interloc_grid grid);
        void move(int delta_x, int delta_y);


        std::string robot_name;

    private:
        // ROS NodeHandle
        ros::NodeHandle n;

        // ROS Topic Publishers
        ros::Publisher interloc_pub;
};