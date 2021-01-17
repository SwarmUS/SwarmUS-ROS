#include "../include/hiveboard_bridge/TCPServer.h"
#include "../include/hiveboard_bridge/TCPServerMonitor.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "hiveboard_bridge");

    TCPServerMonitor monitor;
    TCPServer socket(8080, monitor);

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