#include "hiveboard_bridge/HiveBoardBridge.h"

HiveBoardBridge::HiveBoardBridge(ITCPServer& tcpServer, IHiveMindHostSerializer& serializer,
                                 IHiveMindHostDeserializer& deserializer):
                                 m_tcpServer(tcpServer), m_serializer(serializer), m_deserializer(deserializer){}

void HiveBoardBridge::spin() {
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

void HiveBoardBridge::onConnect(Hook hook) {
    m_tcpServer.onConnect(hook);
}

void HiveBoardBridge::onDisconnect(Hook hook) {
    m_tcpServer.onDisconnect(hook);
}

bool HiveBoardBridge::registerCustomAction(std::string name, CallbackFunction callback) {
    return m_messageHandler.registerCallback(name, callback);
}
