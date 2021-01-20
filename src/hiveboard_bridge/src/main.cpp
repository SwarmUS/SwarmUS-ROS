#include "hiveboard_bridge/TCPServer.h"
#include "hiveboard_bridge/TCPServerMonitor.h"
#include "ros/ros.h"

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
    TCPServer socket(port);

    ROS_INFO("Listening for incoming clients...");
    socket.listen();
    ROS_INFO("Client connected.");

    char sendValue[] = "Hello world from ROS!";
    socket.send(sendValue, strlen(sendValue));
    ROS_INFO("Sent a message from server to client");

    char buf[10] = "";
    char okBuf[3] = "Ok";
    int i = 0;
    while (true) {
        ROS_INFO("%d", i++);

        bzero(buf, 10);
        socket.read(buf, 10, true);
        ROS_INFO("Data: %s", buf);
        socket.send(okBuf, 3);
    }

    ros::spin();

    return 0;
}