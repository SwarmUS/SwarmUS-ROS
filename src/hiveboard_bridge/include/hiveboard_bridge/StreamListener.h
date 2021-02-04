#ifndef CATKIN_ROS_STREAMLISTENER_H
#define CATKIN_ROS_STREAMLISTENER_H

#include <functional>
#include <optional>
#include <thread>
#include "ros/ros.h"
#include "IMessageHandler.h"
#include <common/IProtobufStream.h>
#include <hivemind-host/IHiveMindHostDeserializer.h>
#include <hivemind-host/MessageDTO.h>

class StreamListener {
  public:
    StreamListener(IHiveMindHostDeserializer& deserializer, IMessageHandler& messageHandler);
    ~StreamListener();

protected:
    IHiveMindHostDeserializer& m_deserializer;
    IMessageHandler& m_messageHandler;
    std::thread m_rcvThread;

    void receiveThread();
    void receiveThreadAction();
};

#endif // CATKIN_ROS_STREAMLISTENER_H
