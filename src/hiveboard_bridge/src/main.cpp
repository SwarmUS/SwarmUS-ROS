#include "hiveboard_bridge/MessageHandler.h"
#include "hiveboard_bridge/ReceiveAction.h"
#include "hiveboard_bridge/TCPServer.h"
#include "ros/ros.h"
#include "swarmus_ros_navigation/MoveByMessage.h"
#include <chrono>
#include <functional>
#include <hivemind-host/FunctionCallArgumentDTO.h>
#include <hivemind-host/FunctionCallResponseDTO.h>
#include <hivemind-host/HiveMindHostDeserializer.h>
#include <hivemind-host/HiveMindHostSerializer.h>
#include <hivemind-host/MessageDTO.h>
#include <hivemind-host/ResponseDTO.h>
#include <optional>
#include <thread>

constexpr uint8_t RATE_HZ{2};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hiveboard_bridge");
    ros::NodeHandle nodeHandle;

    std::string robotName = ros::param::param("~ROBOT_NAME", (std::string) "pioneer_0");
    ros::Publisher moveByPublisher = nodeHandle.advertise<swarmus_ros_navigation::MoveByMessage>(
        robotName + "/navigation/moveBy", 1000);

    ros::Subscriber sub;

    int port = ros::param::param("~TCP_SERVER_PORT", 8080);
    TCPServer tcpServer(port);

    ROS_INFO("Listening for incoming clients on port %d...", port);
    tcpServer.listen();
    ROS_INFO("Client connected.");

    HiveMindHostDeserializer deserializer(tcpServer);
    HiveMindHostSerializer serializer(tcpServer);
    MessageHandler messageHandler;

    // Register callbacks
    CallbackFunction moveByCallback = [&](CallbackArgs args, int argsLength) {
        swarmus_ros_navigation::MoveByMessage moveByMessage;

        moveByMessage.distance_x = std::get<float>(args[0].getArgument());
        moveByMessage.distance_y = std::get<float>(args[1].getArgument());

        // Publish on moveby
        moveByPublisher.publish(moveByMessage);

        // TODO Send ack/response over TCP
    };

    messageHandler.registerCallback("moveBy", moveByCallback);

    ReceiveAction receiveAction(deserializer, messageHandler);

    ros::Rate loopRate(RATE_HZ);
    while (ros::ok()) {
        ros::spinOnce();

        receiveAction.fetchAndProcessMessage();

        loopRate.sleep();
    }

    return 0;
}