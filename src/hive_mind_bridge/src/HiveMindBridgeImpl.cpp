#include "hive_mind_bridge/HiveMindBridgeImpl.h"

HiveMindBridgeImpl::HiveMindBridgeImpl(ITCPServer& tcpServer,
                                       IHiveMindHostSerializer& serializer,
                                       IHiveMindHostDeserializer& deserializer) :
    m_tcpServer(tcpServer), m_serializer(serializer), m_deserializer(deserializer) {}

void HiveMindBridgeImpl::spin() {
    if (m_tcpServer.isClientConnected()) {
        MessageDTO message;
        if (m_deserializerFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {

        } else {
            m_deserializerFuture = std::async(std::launch::async, m_deserializer.deserializeFromStream, message);
        }
        if (m_deserializer.deserializeFromStream(message)) {
            // Execute the action
            result = m_messageHandler.handleMessage(message);

            // Send the ack/nack message
            m_serializer.serializeToStream(result.getResponse());


        } else {
            ROS_WARN("The received bytes do not contain a valid message.");
        }

        // Check if return values are ready
        // todo put this in a queue to manage more than one at a time.
        ROS_INFO("Checking callback status...");

        ROS_WARN("SPIN FUTURE : %d", result.getReturnValues().valid());
        std::future_status status = result.getReturnValues().wait_for(std::chrono::microseconds (1));
        ROS_INFO("Done.");
        if (status == std::future_status::ready) {
            // create msg with apropriate request
            ROS_INFO("CALLBACK HAS RETURNED");

            // send msg.
        }

        ROS_INFO("After expected return");

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