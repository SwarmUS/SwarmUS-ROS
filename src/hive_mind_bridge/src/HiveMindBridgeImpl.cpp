#include "hive_mind_bridge/HiveMindBridgeImpl.h"

HiveMindBridgeImpl::HiveMindBridgeImpl(ITCPServer& tcpServer,
                                       IHiveMindHostSerializer& serializer,
                                       IHiveMindHostDeserializer& deserializer) :
    m_tcpServer(tcpServer), m_serializer(serializer), m_deserializer(deserializer) {}

void HiveMindBridgeImpl::spin() {
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

void HiveMindBridgeImpl::onConnect(std::function<void()> hook) { m_tcpServer.onConnect(hook); }

void HiveMindBridgeImpl::onDisconnect(std::function<void()> hook) {
    m_tcpServer.onDisconnect(hook);
}

bool HiveMindBridgeImpl::registerCustomAction(std::string name,
                                              CallbackFunction callback,
                                              CallbackArgsManifest manifest) {
    return m_messageHandler.registerCallback(name, callback, manifest);
}

bool HiveMindBridgeImpl::registerCustomAction(std::string name, CallbackFunction callback) {
    return m_messageHandler.registerCallback(name, callback);
}