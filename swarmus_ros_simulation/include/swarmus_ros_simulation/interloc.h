#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include <tf/transform_listener.h>
#include <string>

#include <sstream>

// TODO change std::String to std_msgs::String

class Interloc {
    public:
        Interloc(std::string new_robot_name);
        float getDistanceFrom(float x, float y);
        float getAnglefrom(float x, float y);
        void publish(std_msgs::String msg);
        void move(int delta_x, int delta_y);
        std::string robot_name;

    private:
        // ROS NodeHandle
        ros::NodeHandle n;

        // ROS Topic Publishers
        ros::Publisher interloc_pub;

        // ROS Topic listeners
//        tf::TransformListener ;
        
        // Variables
        float pos_x = 0.0;
        float pos_y = 0.0;
};