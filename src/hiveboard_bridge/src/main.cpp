#include "hiveboard_bridge/MessageHandler.h"
#include "hiveboard_bridge/ReceiveAction.h"
#include "hiveboard_bridge/TCPServer.h"
#include "ros/ros.h"
#include "swarmus_ros_navigation/MoveByMessage.h"
#include <chrono>
#include <functional>
#include <hivemind-host/FunctionCallArgumentDTO.h>
#include <hivemind-host/FunctionCallRequestDTO.h>
#include <hivemind-host/FunctionCallResponseDTO.h>
#include <hivemind-host/HiveMindHostDeserializer.h>
#include <hivemind-host/HiveMindHostSerializer.h>
#include <hivemind-host/MessageDTO.h>
#include <hivemind-host/RequestDTO.h>
#include <hivemind-host/ResponseDTO.h>
#include <optional>
#include <thread>

#define DEFAULT_TCP_SERVER_PORT 8080
#define DEFAULT_ROBOT_NAME "pioneer_0"
static const uint8_t RATE_HZ{2};

int getTcpServerPort() {
    int port;

    if (!ros::param::get("~TCP_SERVER_PORT", port)) {
        ROS_INFO("No TCP_SERVER_PORT param was given. Using default value %d",
                 DEFAULT_TCP_SERVER_PORT);
        port = DEFAULT_TCP_SERVER_PORT;
    }

    return port;
}

std::string getRobotName() {
    std::string robotName;

    if (!ros::param::get("~ROBOT_NAME", robotName)) {
        ROS_INFO("No ROBOT_NAME parameter was given. Using default value %s", DEFAULT_ROBOT_NAME);
        robotName = DEFAULT_ROBOT_NAME;
    }

    return robotName;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "hiveboard_bridge");
    ros::NodeHandle nodeHandle;
    ros::Publisher moveByPublisher = nodeHandle.advertise<swarmus_ros_navigation::MoveByMessage>(
        getRobotName() + "/navigation/moveBy", 1000);
    ros::Subscriber sub;

    int port = getTcpServerPort();
    TCPServer tcpServer(port);

    ROS_INFO("Listening for incoming clients...");
    tcpServer.listen();
    ROS_INFO("Client connected.");

    HiveMindHostDeserializer deserializer(tcpServer);
    HiveMindHostSerializer serializer(tcpServer);
    MessageHandler messageHandler;

    // Register callbacks
    CallbackFunction moveByCallback = [&](CallbackArgs args, int argsLength) {
        swarmus_ros_navigation::MoveByMessage moveByMessage;

        moveByMessage.distance_x = std::get<float>(args[0].getArgument());
        moveByMessage.distance_y = std::get<int64_t>(args[1].getArgument());

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