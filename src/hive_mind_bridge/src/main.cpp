#include "hiveboard_bridge/HiveMindBridge.h"
#include "hiveboard_bridge/MessageHandler.h"
#include "ros/ros.h"
#include "swarmus_ros_navigation/MoveByMessage.h"
#include <hivemind-host/FunctionCallArgumentDTO.h>
#include <optional>

constexpr uint8_t RATE_HZ{2};
constexpr uint32_t compoundId{1}; // TODO find a way for the HiveMind and the robot to share this ID

int main(int argc, char** argv) {
    ros::init(argc, argv, "hiveboard_bridge");
    ros::NodeHandle nodeHandle;

    std::string robotName = ros::param::param("~ROBOT_NAME", std::string("pioneer_0"));
    ros::Publisher moveByPublisher = nodeHandle.advertise<swarmus_ros_navigation::MoveByMessage>(
        robotName + "/navigation/moveBy", 1000);

    ros::Subscriber sub;

    int port = ros::param::param("~TCP_SERVER_PORT", 8080);
    HiveMindBridge bridge(port);

    // Register custom actions
    CallbackFunction moveByCallback = [&](CallbackArgs args, int argsLength) {
        swarmus_ros_navigation::MoveByMessage moveByMessage;

        moveByMessage.distance_x = std::get<float>(args[0].getArgument());
        moveByMessage.distance_y = std::get<float>(args[1].getArgument());

        // Publish on moveby
        moveByPublisher.publish(moveByMessage);
    };
    bridge.registerCustomAction("moveBy", moveByCallback);

    // Register event hooks
    bridge.onConnect([]() { ROS_INFO("Client connected."); });

    bridge.onDisconnect([]() { ROS_INFO("Client disconnected."); });

    ros::Rate loopRate(RATE_HZ);
    while (ros::ok()) {
        ros::spinOnce();

        bridge.spin();

        loopRate.sleep();
    }

    return 0;
}