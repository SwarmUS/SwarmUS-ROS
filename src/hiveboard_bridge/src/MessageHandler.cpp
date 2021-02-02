#include "hiveboard_bridge/MessageHandler.h"

MessageHandler::MessageHandler() { }

MessageHandler::~MessageHandler() { }

bool MessageHandler::handleMessage(MessageDTO message) {
    // todo Interpret the message
    const std::variant<std::monostate, RequestDTO>& request = message.getMessage();

    if (std::holds_alternative<RequestDTO>(request)) {
        std::variant<std::monostate, FunctionCallRequestDTO> functionCallRequest = std::get<RequestDTO>(request).getRequest();

        if (std::holds_alternative<FunctionCallRequestDTO>(functionCallRequest)) {
            ROS_INFO("Function name: %s", std::get<FunctionCallRequestDTO>(functionCallRequest).getFunctionName());
            return true;
        }
    }
     return false;

//    auto callback = getCallback(message);
//
//    // Call the right callback
//    if (callback) {
//        callback.value();
//        return true;
//    }
//
//    return false;
}

void MessageHandler::registerCallback(std::string name, std::function<void()> callback) {
    m_callbacks[name] = callback;
}

std::optional<std::function<void()>> MessageHandler::getCallback(std::string name) {
    auto callback = m_callbacks.find(name);
    if (callback != m_callbacks.end()) {
        return callback->second;
    }

    return {};
}