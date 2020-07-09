#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>

#include <sstream>

class Interloc {
    public:
        Interloc();
        float getDistanceFrom(int x, int y);
        float getAnglefrom(int x, int y);
        void publish(std_msgs::String msg);
        void move(int delta_x, int delta_y);

    private:
        // ROS NodeHandle
        ros::NodeHandle n;

        // ROS Topic Publishers
        ros::Publisher interloc_pub;
        
        // Variables
        float pos_x = 0.0;
        float pos_y = 0.0;
};