#include "TCPServer.h"
#include "TCPServerMonitor.h"
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
    int i = 0;
    while (true) {
        ROS_INFO("%d", i++);

        socket.read(buf, 2, true);
        ROS_INFO("Header: %s", buf);
        socket.send("OK", 3);

        bzero(buf, 10);
        socket.read(buf, 10, true);
        ROS_INFO("Data: %s", buf);
    }

    ros::spin();

    return 0;
}