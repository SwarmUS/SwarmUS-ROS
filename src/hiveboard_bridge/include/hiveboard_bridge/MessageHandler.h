#ifndef CATKIN_ROS_MESSAGEHANDLER_H
#define CATKIN_ROS_MESSAGEHANDLER_H

#include "IMessageHandler.h"

class MessageHandler : public IMessageHandler {
  public:
    MessageHandler();
    ~MessageHandler();

    bool handleMessage(MessageDTO message) override;

    bool registerCallback(std::string name, CallbackFunction callback) override;

    std::optional<CallbackFunction> getCallback(std::string name) override;

  private:
    CallbackMap m_callbacks;
};

#endif // CATKIN_ROS_MESSAGEHANDLER_H
