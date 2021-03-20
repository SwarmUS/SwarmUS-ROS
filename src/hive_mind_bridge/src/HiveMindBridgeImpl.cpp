#include "hive_mind_bridge/HiveMindBridgeImpl.h"

HiveMindBridgeImpl::HiveMindBridgeImpl(ITCPServer& tcpServer,
                                       IHiveMindHostSerializer& serializer,
                                       IHiveMindHostDeserializer& deserializer,
                                       IMessageHandler& messageHandler,
                                       IThreadSafeQueue<MessageDTO>& queue) :
    m_tcpServer(tcpServer), m_serializer(serializer), m_deserializer(deserializer), m_messageHandler(messageHandler), m_inboundQueue(queue) {}

void HiveMindBridgeImpl::spin() {
    if (isTCPClientConnected()) {
        if (!m_inboundQueue.empty()) {
            // Execute the action
            MessageHandlerResult result = m_messageHandler.handleMessage(m_inboundQueue.front());
            m_inboundQueue.pop();

            m_resultQueue.push_back(result);

            // Send the ack/nack message
            m_serializer.serializeToStream(result.getResponse());
        }

        for (auto result : m_resultQueue) {
            if (result.getFuture().wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                sendReturn(result);

                m_resultQueue.pop_front();
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

void HiveMindBridgeImpl::inboundThread() {
    while (isTCPClientConnected()) {
        MessageDTO message;
        if (m_deserializer.deserializeFromStream(message)) {
            m_inboundQueue.push(message);
        }
    }
}

bool HiveMindBridgeImpl::isTCPClientConnected() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_tcpServer.isClientConnected();
}

void HiveMindBridgeImpl::sendReturn(MessageHandlerResult result) {
    std::optional<CallbackArgs> argsOpt = result.getFuture().get();

    // Send a return only if there is a return value
    if (argsOpt.has_value()) {
        CallbackArgs args = argsOpt.value();
        MessageDTO returnMessage = MessageUtils::createFunctionCallRequest(
            result.getMessageDestinationId(), // swap source and dest since we return to the sender
            result.getMessageSourceId(),
            99, // TODO what should I put here?
            result.getSourceModule(), // swap source and dest since we return to the sender
            result.getReturnCallbackName(), args);

        m_serializer.serializeToStream(returnMessage);
    }
}