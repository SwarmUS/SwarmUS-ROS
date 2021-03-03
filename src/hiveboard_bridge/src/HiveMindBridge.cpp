#include "hiveboard_bridge/HiveMindBridge.h"

HiveBoardBridge::HiveBoardBridge(int tcpPort) :
    m_tcpServer(tcpPort), m_deserializer(m_tcpServer), m_serializer(m_tcpServer) {
    m_bridge = std::make_unique<HiveBoardBridgeImpl>(m_tcpServer, m_serializer, m_deserializer);
}

void HiveBoardBridge::spin() { m_bridge->spin(); }

void HiveBoardBridge::onConnect(std::function<void()> hook) { m_bridge->onConnect(hook); }

void HiveBoardBridge::onDisconnect(std::function<void()> hook) { m_bridge->onDisconnect(hook); }

bool HiveBoardBridge::registerCustomAction(std::string name, CallbackFunction callback) {
    return m_bridge->registerCustomAction(name, callback);
}
