#ifndef CATKIN_ROS_STREAMLISTENER_H
#define CATKIN_ROS_STREAMLISTENER_H

#include "ReceiveThreadAction.h"
#include "ros/ros.h"
#include <common/IProtobufStream.h>
#include <functional>
#include <hivemind-host/MessageDTO.h>
#include <optional>
#include <thread>

/**
 * A class that listens on a stream and performs some actions when messages arrive
 */
class StreamListener {
  public:
    /**
     * Default constructor
     * @param receiveThreadAction The action to be performed in the receive thread
     */
    StreamListener(ReceiveThreadAction receiveThreadAction);

    ~StreamListener();

  protected:
    ReceiveThreadAction m_receiveThreadAction;

    std::thread m_rcvThread;
    void receiveThread();
};

#endif // CATKIN_ROS_STREAMLISTENER_H
