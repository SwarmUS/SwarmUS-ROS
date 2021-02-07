#include "hiveboard_bridge/ReceiveAction.h"

ReceiveAction::ReceiveAction(IHiveMindHostDeserializer& deserializer,
                                         IMessageHandler& messageHandler) :
    m_deserializer(deserializer), m_messageHandler(messageHandler) {}

void ReceiveAction::doAction() {
    std::variant<std::monostate, MessageDTO> message = m_deserializer.deserializeFromStream();
    if (std::holds_alternative<MessageDTO>(message)) {
        if (!m_messageHandler.handleMessage(std::get<MessageDTO>(message))) {
            ROS_WARN("Message handling failed.");
        }
    } else {
        ROS_WARN("The received bytes do not contain a valid message.");
    }
}