#include "hiveboard_bridge/ReceiveAction.h"

ReceiveAction::ReceiveAction(IHiveMindHostDeserializer& deserializer,
                             IHiveMindHostSerializer& serializer,
                             IMessageHandler& messageHandler) :
    m_deserializer(deserializer), m_serializer(serializer), m_messageHandler(messageHandler) {}

void ReceiveAction::fetchAndProcessMessage() {
    MessageDTO message;
    if (m_deserializer.deserializeFromStream(message)) {
        // Execute the action
        MessageDTO responseMessage = m_messageHandler.handleMessage(message);

        // Send the ack/nack message
        m_serializer.serializeToStream(responseMessage);
    } else {
        ROS_WARN("The received bytes do not contain a valid message.");
    }
}