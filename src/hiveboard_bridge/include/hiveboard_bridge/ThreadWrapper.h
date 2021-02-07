#ifndef CATKIN_ROS_THREADWRAPPER_H
#define CATKIN_ROS_THREADWRAPPER_H

#include "ReceiveAction.h"
#include "ros/ros.h"
#include <common/IProtobufStream.h>
#include <functional>
#include <hivemind-host/MessageDTO.h>
#include <optional>
#include <thread>

/**
 * A class that listens on a stream and performs some actions when messages arrive
 */
class ThreadWrapper {
  public:
    /**
     * Construct a ThreadWrapper object
     * @param ReceiveAction The action to be performed in the receive thread
     * \param sleepTimeMs The sleep time between each cycle of the receive thread
     */
    ThreadWrapper(ReceiveAction ReceiveAction, int sleepTimeMs);

    ~ThreadWrapper();

  protected:
    ReceiveAction m_receiveAction;
    bool m_threadShouldRun = true;
    std::thread m_rcvThread;
    int m_sleepTimeMs;
    void receiveThread();
};

#endif // CATKIN_ROS_THREADWRAPPER_H
