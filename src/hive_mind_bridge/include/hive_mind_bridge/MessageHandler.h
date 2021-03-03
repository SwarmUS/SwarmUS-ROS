#ifndef CATKIN_ROS_MESSAGEHANDLER_H
#define CATKIN_ROS_MESSAGEHANDLER_H

#include "IMessageHandler.h"
#include "hive_mind_bridge/MessageUtils.h"

class MessageHandler : public IMessageHandler {
  public:
    MessageHandler();
    ~MessageHandler();

    MessageDTO handleMessage(MessageDTO message) override;

    bool registerCallback(std::string name, CallbackFunction callback) override;

    std::optional<CallbackFunction> getCallback(const std::string& name) override;

  private:
    CallbackMap m_callbacks;
};

#endif // CATKIN_ROS_MESSAGEHANDLER_H
