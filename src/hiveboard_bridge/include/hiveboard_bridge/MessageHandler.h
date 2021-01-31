#ifndef CATKIN_ROS_MESSAGEHANDLER_H
#define CATKIN_ROS_MESSAGEHANDLER_H

#include <functional>
#include <unordered_map>
#include <optional>
#include <ros/ros.h>

typedef std::unordered_map<std::string, std::function<void()>> CallbackMap;

class MessageHandler {
public:
    MessageHandler();
    ~MessageHandler();

    /**
     * Parse a message and execute the appropriate callback.
     * @param message the message to parse.
     * @return True if the appropriate callback was found and executed, false otherwise
     */
    bool handleMessage(std::string message); // todo change this for a real message type

    /**
     * Register a callback
     * @param name Key of the callback
     * @param callback Callback function
     */
    void registerCallback(std::string name, std::function<void()> callback);

    /**
     * Get an instance of a callback, if it exists.
     * @param name Key under which the callback was registered
     * @return The callback function if it exists.
     */
    std::optional<std::function<void()>> getCallback(std::string name);

private:
    CallbackMap m_callbacks;
};


#endif //CATKIN_ROS_MESSAGEHANDLER_H
