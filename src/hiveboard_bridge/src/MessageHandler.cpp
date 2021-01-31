#include "hiveboard_bridge/MessageHandler.h"

MessageHandler::MessageHandler() { }

MessageHandler::~MessageHandler() { }

bool MessageHandler::handleMessage(std::string message) {
    // todo Parse the message

    auto callback = getCallback(message);

    // Call the right callback
    if (callback) {
        callback.value();
        return true;
    }

    return false;
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