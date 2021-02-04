#include "hiveboard_bridge/MessageHandler.h"

MessageHandler::MessageHandler() {}

MessageHandler::~MessageHandler() {}

bool MessageHandler::handleMessage(MessageDTO message) {
    std::variant<std::monostate, RequestDTO> request = message.getMessage();

    if (std::holds_alternative<RequestDTO>(request)) {
        std::variant<std::monostate, FunctionCallRequestDTO> functionCallRequest =
            std::get<RequestDTO>(request).getRequest();

        if (std::holds_alternative<FunctionCallRequestDTO>(functionCallRequest)) {
            FunctionCallRequestDTO function = std::get<FunctionCallRequestDTO>(functionCallRequest);
            std::string functionName = function.getFunctionName();
            CallbackArgs functionArgs = function.getArguments();
            uint16_t argsLength = function.getArgumentsLength();

            auto callback = getCallback(functionName);

            // Call the right callback
            if (callback) {
                callback.value()(functionArgs);
                return true;
            } else {
                ROS_WARN("Function name \"%s\" was not registered as a callback",
                         functionName.c_str());
            }
        }
    }
    return false;
}

void MessageHandler::registerCallback(std::string name, CallbackFunction callback) {
    m_callbacks[name] = callback;
}

std::optional<CallbackFunction> MessageHandler::getCallback(std::string name) {
    auto callback = m_callbacks.find(name);
    if (callback != m_callbacks.end()) {
        return callback->second;
    }

    return {};
}