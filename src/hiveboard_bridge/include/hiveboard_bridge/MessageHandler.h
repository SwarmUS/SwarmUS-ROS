#ifndef CATKIN_ROS_MESSAGEHANDLER_H
#define CATKIN_ROS_MESSAGEHANDLER_H

#include <functional>
#include <hivemind-host/FunctionCallArgumentDTO.h>
#include <hivemind-host/FunctionCallRequestDTO.h>
#include <hivemind-host/FunctionCallResponseDTO.h>
#include <hivemind-host/MessageDTO.h>
#include <hivemind-host/RequestDTO.h>
#include <optional>
#include <ros/ros.h>
#include <unordered_map>
#include <variant>

typedef std::array<FunctionCallArgumentDTO,
                   FunctionCallRequestDTO::FUNCTION_CALL_ARGUMENTS_MAX_LENGTH>
    CallbackArgs;
typedef std::function<void(CallbackArgs)> CallbackFunction;
typedef std::unordered_map<std::string, CallbackFunction> CallbackMap;

class MessageHandler {
  public:
    MessageHandler();
    ~MessageHandler();

    /**
     * Parse a message and execute the appropriate callback.
     * @param message the message to parse.
     * @return True if the appropriate callback was found and executed, false otherwise
     */
    bool handleMessage(MessageDTO message);

    /**
     * Register a callback
     * @param name Key of the callback
     * @param callback Callback function
     */
    void registerCallback(std::string name, CallbackFunction callback);

    /**
     * Get an instance of a callback, if it exists.
     * @param name Key under which the callback was registered
     * @return The callback function if it exists.
     */
    std::optional<CallbackFunction> getCallback(std::string name);

  private:
    CallbackMap m_callbacks;
};

#endif // CATKIN_ROS_MESSAGEHANDLER_H
