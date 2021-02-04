#include "hiveboard_bridge/StreamListener.h"

StreamListener::StreamListener(IHiveMindHostDeserializer& deserializer, IMessageHandler& messageHandler) :
    m_deserializer(deserializer), m_messageHandler(messageHandler) {
    m_rcvThread = std::thread(&StreamListener::receiveThread, this);
}

StreamListener::~StreamListener() {
    if (m_rcvThread.joinable()) {
        m_rcvThread.join();
    }
}

void StreamListener::receiveThread() {
    while (true) {
        receiveThreadAction();
    }
}

void StreamListener::receiveThreadAction() {
    std::variant<std::monostate, MessageDTO> message = m_deserializer.deserializeFromStream();
    if (std::holds_alternative<MessageDTO>(message)) {
        if (!m_messageHandler.handleMessage(std::get<MessageDTO>(message))) {
            ROS_WARN("Message handling failed.");
        }
    }
}
