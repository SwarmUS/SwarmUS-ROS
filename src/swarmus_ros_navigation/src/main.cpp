#include "Navigation.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    //  Initialize rosbuzz node
    ros::init(argc, argv, "navigation");
    std::shared_ptr<ros::NodeHandle> nodeHandle(new ros::NodeHandle("~"));
    
    // Initialize and start node execution
    Navigation navigation(nodeHandle);
    navigation.execute();

    return 0;
}