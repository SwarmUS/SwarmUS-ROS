#include "ros/ros.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "hiveboard_bridge");

    ros::spin();

    return 0;
}