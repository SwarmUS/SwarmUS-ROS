#ifndef CATKIN_ROS_STREAMLISTENER_H
#define CATKIN_ROS_STREAMLISTENER_H

#include "ros/ros.h"
#include <thread>
#include <optional>
#include <mutex>
#include <functional>
#include "TCPServer.h" // Todo replace this with pheromones?
#include "MessageHandler.h"
#include <hivemind-host/MessageDTO.h>

class StreamListener {
public:

    StreamListener(int port, MessageHandler messageHandler);
    ~StreamListener();

private:
    TCPServer m_socket; // This will be replaced.
    std::thread m_rcvThread;
    MessageHandler m_messageHandler;

    void receiveThread();
};

#endif //CATKIN_ROS_STREAMLISTENER_H
