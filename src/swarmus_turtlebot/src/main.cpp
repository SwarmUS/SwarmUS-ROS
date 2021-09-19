#include "ros/ros.h"
#include "swarmus_turtlebot/Navigation.hpp"
#include <thread>

static const uint8_t RATE_HZ{1};

static const uint32_t QUEUE_SIZE{1000};
const std::string ROBOT_BASE_FRAME{"base_footprint"};
const std::string MOVEBY_TOPIC{"navigation/moveBy"};
const std::string MOVEBASE_GOAL_TOPIC{"move_base_simple/goal"};

void navigationLoop(Navigation* navigation, ros::Rate loopRate) {
    while(true) {
        navigation->execute();

        loopRate.sleep();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "swarmus_turtlebot_bridge");

    std::shared_ptr<ros::NodeHandle> nodeHandle(new ros::NodeHandle(""));

    Navigation navigation(nodeHandle);

    ros::Rate loopRate(RATE_HZ);

    std::string moveByTopic =
            nodeHandle->param("moveByTopic", std::string("/navigation/moveBy"));
    ros::Publisher moveByPublisher =
            nodeHandle->advertise<swarmus_turtlebot::MoveByMessage>(moveByTopic, 1000);

    std::thread navigationThread(navigationLoop, &navigation, loopRate);

    swarmus_turtlebot::MoveByMessage moveByMessage;

    moveByMessage.distance_x = 1;
    moveByMessage.distance_y = 0;

    moveByPublisher.publish(moveByMessage);

    while (ros::ok()) {
        ros::spinOnce();

//        navigation.execute();

        loopRate.sleep();
    }


    return 0;
}