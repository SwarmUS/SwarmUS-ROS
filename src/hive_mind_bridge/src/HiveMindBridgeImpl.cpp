#include "hive_mind_bridge/HiveMindBridgeImpl.h"

HiveMindBridgeImpl::HiveMindBridgeImpl(ITCPServer& tcpServer,
                                       IHiveMindHostSerializer& serializer,
                                       IHiveMindHostDeserializer& deserializer) :
    m_tcpServer(tcpServer), m_serializer(serializer), m_deserializer(deserializer) {
}

void HiveMindBridgeImpl::inboundThread() {
    while (isTCPClientConnected()) {
        MessageDTO message;
        if (m_deserializer.deserializeFromStream(message)) {
            m_inboundQueue.push(message);
        }
    }

    ROS_INFO("inbound thread will terminate");
}

bool HiveMindBridgeImpl::isTCPClientConnected() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_tcpServer.isClientConnected();
}

void HiveMindBridgeImpl::spin() {
    if (isTCPClientConnected()) {
        MessageDTO message;
        if (!m_inboundQueue.empty()) {
            // Execute the action
            result = m_messageHandler.handleMessage(m_inboundQueue.front());
            m_inboundQueue.pop();

            m_futuresQueue.push_back(result.getFuture());

            // Send the ack/nack message
            m_serializer.serializeToStream(result.getResponse());
        }

        for (auto item : m_futuresQueue) {
            if (item.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                // create msg with apropriate request
                ROS_INFO("CALLBACK HAS RETURNED");

                // send msg.

                m_futuresQueue.pop_front();
            }
        }
    } else {
        if (m_inboundThread.joinable()) {
            m_inboundThread.join();
        }

        m_tcpServer.listen();

        m_inboundThread = std::thread(&HiveMindBridgeImpl::inboundThread, this);
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