#include "ros/ros.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "hivemind_bridge");

    ROS_INFO("NO HIVEMIND");

    ros::spin();

    return 0;
}
