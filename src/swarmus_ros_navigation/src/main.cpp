#include "Navigation.hpp"
#include <ros/ros.h>

static const uint8_t RATE_HZ{20};

int main(int argc, char** argv) {
    //  Initialize rosbuzz node
    ros::init(argc, argv, "navigation");

    std::shared_ptr<ros::NodeHandle> nodeHandle(new ros::NodeHandle(""));

    // Initialize and start node execution
    Navigation navigation(nodeHandle);

    ros::Rate loopRate(RATE_HZ);

    while (ros::ok()) {
        ros::spinOnce();

        navigation.execute();

        loopRate.sleep();
    }

    return 0;
}