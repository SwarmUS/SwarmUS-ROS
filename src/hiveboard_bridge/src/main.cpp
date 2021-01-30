#include "hiveboard_bridge/TCPServer.h"
#include "hiveboard_bridge/TCPServerMonitor.h"
#include "hiveboard_bridge/StreamListener.h"
#include "hiveboard_bridge/MessageHandler.h"
#include "ros/ros.h"
#include <thread>
#include <optional>
#include <mutex>
#include <functional>
#include <chrono> // TODO remove this

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

    MessageHandler messageHandler;
    std::function<void()> testFunction = []() {
        ROS_INFO("Test printing from a function pointer");
    };
    messageHandler.registerCallback("Hello", testFunction);

    StreamListener streamListener(port, messageHandler);

    ros::spin();
    return 0;
}