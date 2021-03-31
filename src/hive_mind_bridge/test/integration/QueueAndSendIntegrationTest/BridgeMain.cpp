#include "hive_mind_bridge/Callback.h"
#include "hive_mind_bridge/HiveMindBridge.h"
#include "hive_mind_bridge/MessageHandler.h"
#include "ros/ros.h"
#include "swarmus_ros_navigation/MoveByMessage.h"
#include <hivemind-host/FunctionCallArgumentDTO.h>
#include <optional>

constexpr uint8_t RATE_HZ{1};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hive_mind_bridge");
    ros::NodeHandle nodeHandle;

    std::string robotName = ros::param::param("~ROBOT_NAME", std::string("pioneer_0"));
    int port = ros::param::param("~TCP_SERVER_PORT", 8080);

    HiveMindBridge bridge(port);

    // Register event hooks
    bridge.onConnect([]() { ROS_INFO("Client connected."); });

    bridge.onDisconnect([]() { ROS_INFO("Client disconnected."); });

    ros::Rate loopRate(RATE_HZ);
    while (ros::ok()) {
        ros::spinOnce();

        bridge.queueAndSend(MessageUtils::createFunctionCallRequest(42, 42, 1, UserCallTargetDTO::UNKNOWN, "someRemoteCallback"));

        bridge.spin();

        loopRate.sleep();
    }

    return 0;
}