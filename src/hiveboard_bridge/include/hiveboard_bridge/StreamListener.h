#ifndef CATKIN_ROS_STREAMLISTENER_H
#define CATKIN_ROS_STREAMLISTENER_H

#include "MessageHandler.h"
#include "TCPServer.h" // Todo replace this with pheromones?
#include "ros/ros.h"
#include <functional>
#include <hivemind-host/MessageDTO.h>
#include <mutex>
#include <optional>
#include <thread>

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

#endif // CATKIN_ROS_STREAMLISTENER_H
