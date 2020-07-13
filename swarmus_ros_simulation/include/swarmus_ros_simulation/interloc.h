#include "ros/ros.h"
#include "std_msgs/String.h"
#include "swarmus_ros_simulation/Interloc_msg.h"
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
        void publish(std_msgs::String msg);
        void move(int delta_x, int delta_y);
        void getRobotList(ros::NodeHandle nh);


        std::string robot_name;
        std::vector<std::string> robot_list;
        /*std::map<std::string, 
        swarmus_ros_simulation::Interloc_msg> robot_list;*/           //TODO mettre la bonne classe

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