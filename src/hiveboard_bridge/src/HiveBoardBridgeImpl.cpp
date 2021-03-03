
#include "hiveboard_bridge/HiveBoardBridgeImpl.h"

HiveBoardBridgeImpl::HiveBoardBridgeImpl(ITCPServer& tcpServer,
                                         IHiveMindHostSerializer& serializer,
                                         IHiveMindHostDeserializer& deserializer) :
    m_tcpServer(tcpServer), m_serializer(serializer), m_deserializer(deserializer) {}

void HiveBoardBridgeImpl::spin() {
    if (m_tcpServer.isClientConnected()) {
        MessageDTO message;
        if (m_deserializer.deserializeFromStream(message)) {
            // Execute the action
            MessageDTO responseMessage = m_messageHandler.handleMessage(message);

            // Send the ack/nack message
            m_serializer.serializeToStream(responseMessage);
        } else {
            ROS_WARN("The received bytes do not contain a valid message.");
        }
    } else {
        m_tcpServer.listen();
    }
}

void HiveBoardBridgeImpl::onConnect(std::function<void()> hook) { m_tcpServer.onConnect(hook); }

void HiveBoardBridgeImpl::onDisconnect(std::function<void()> hook) {
    m_tcpServer.onDisconnect(hook);
}

bool HiveBoardBridgeImpl::registerCustomAction(std::string name, CallbackFunction callback) {
    return m_messageHandler.registerCallback(name, callback);
}
