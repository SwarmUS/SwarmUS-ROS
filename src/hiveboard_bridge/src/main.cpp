#include "hiveboard_bridge/MessageHandler.h"
#include "hiveboard_bridge/ThreadWrapper.h"
#include "hiveboard_bridge/TCPServer.h"
#include "ros/ros.h"
#include <functional>
#include <hivemind-host/FunctionCallArgumentDTO.h>
#include <hivemind-host/FunctionCallRequestDTO.h>
#include <hivemind-host/FunctionCallResponseDTO.h>
#include <hivemind-host/HiveMindHostDeserializer.h>
#include <hivemind-host/MessageDTO.h>
#include <hivemind-host/RequestDTO.h>
#include <mutex>
#include <optional>
#include <thread>

#define DEFAULT_TCP_SERVER_PORT 8080

int getTcpServerPort() {
    int port;

    if (!ros::param::get("~TCP_SERVER_PORT", port)) {
        ROS_INFO("No TCP_SERVER_PORT param was given. Using default value %d",
                 DEFAULT_TCP_SERVER_PORT);
        port = DEFAULT_TCP_SERVER_PORT;
    }

    return port;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "hiveboard_bridge");

    int port = getTcpServerPort();
    TCPServer tcpServer(port);

    ROS_INFO("Listening for incoming clients...");
    tcpServer.listen();
    ROS_INFO("Client connected.");

    HiveMindHostDeserializer deserializer(tcpServer);
    MessageHandler messageHandler;

    ReceiveAction receiveAction(deserializer, messageHandler);

    ThreadWrapper ThreadWrapper(receiveAction, 500);

    ros::spin();
    return 0;
}