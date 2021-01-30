#ifndef CATKIN_ROS_MESSAGEHANDLER_H
#define CATKIN_ROS_MESSAGEHANDLER_H

#include <functional>
#include <unordered_map>
#include <ros/ros.h>

typedef std::unordered_map<std::string, std::function<void()>> CallbackMap;

class MessageHandler {
public:
    MessageHandler();
    ~MessageHandler();

    void handleMessage(std::string message);
    void registerCallback(std::string name, std::function<void()> callback);

private:
    CallbackMap m_callbacks;
};


#endif //CATKIN_ROS_MESSAGEHANDLER_H
