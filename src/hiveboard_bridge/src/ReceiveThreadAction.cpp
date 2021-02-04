#include "hiveboard_bridge/ReceiveThreadAction.h"

ReceiveThreadAction::ReceiveThreadAction(IHiveMindHostDeserializer& deserializer,
                                         IMessageHandler& messageHandler) :
    m_deserializer(deserializer), m_messageHandler(messageHandler) {}

void ReceiveThreadAction::doAction() {
    std::variant<std::monostate, MessageDTO> message = m_deserializer.deserializeFromStream();
    if (std::holds_alternative<MessageDTO>(message)) {
        if (!m_messageHandler.handleMessage(std::get<MessageDTO>(message))) {
            ROS_WARN("Message handling failed.");
        }
    }
}