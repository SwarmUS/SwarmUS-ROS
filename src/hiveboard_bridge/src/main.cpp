#include "ros/ros.h"
#include "TCPSocket.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "hiveboard_bridge");

    ROS_INFO("Hello.");

    TCPSocket socket(8080);

    ROS_INFO("Listening for incoming clients...");
    socket.listen(1);
    ROS_INFO("Client connected.");

    char sendValue[] = "Hello world from ROS!";
    socket.send(strlen(sendValue), sendValue);
    ROS_INFO("Sent a message from server to client");

    while(ros::ok()) {
        socket.loop();
    }

    ros::spin();

    return 0;
}