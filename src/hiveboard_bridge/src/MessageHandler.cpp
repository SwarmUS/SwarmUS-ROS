#include "hiveboard_bridge/MessageHandler.h"

MessageHandler::MessageHandler() { }

MessageHandler::~MessageHandler() { }

void MessageHandler::handleMessage(std::string message) {
    // Parse the message
    if (message.compare("Hello") == 0) {
        auto callback = m_callbacks.find("Hello");
        if (callback != m_callbacks.end()) {
            callback->second();
        }
    }
    // Call the right callback
}

void MessageHandler::registerCallback(std::string name, std::function<void()> callback) {
    m_callbacks[name] = callback;
}