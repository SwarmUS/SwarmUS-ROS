#ifndef HIVEBOARD_BRIDGE_IMESSAGEHANDLER_H
#define HIVEBOARD_BRIDGE_IMESSAGEHANDLER_H

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

class IMessageHandler {
public:
    virtual ~IMessageHandler() = default;

    /**
     * Parse a message and execute the appropriate callback.
     * @param message the message to parse.
     * @return True if the appropriate callback was found and executed, false otherwise
     */
    virtual bool handleMessage(MessageDTO message) = 0;

    /**
     * Register a callback
     * @param name Key of the callback
     * @param callback Callback function
     */
    virtual void registerCallback(std::string name, CallbackFunction callback) = 0;

    /**
     * Get an instance of a callback, if it exists.
     * @param name Key under which the callback was registered
     * @return The callback function if it exists.
     */
    virtual std::optional<CallbackFunction> getCallback(std::string name) = 0;
};

#endif //HIVEBOARD_BRIDGE_IMESSAGEHANDLER_H