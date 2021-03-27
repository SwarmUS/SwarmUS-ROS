#include "hive_mind_bridge/HiveMindBridgeImpl.h"

HiveMindBridgeImpl::HiveMindBridgeImpl(ITCPServer& tcpServer,
                                       IHiveMindHostSerializer& serializer,
                                       IHiveMindHostDeserializer& deserializer,
                                       IMessageHandler& messageHandler,
                                       IThreadSafeQueue<MessageDTO>& queue) :
    m_tcpServer(tcpServer),
    m_serializer(serializer),
    m_deserializer(deserializer),
    m_messageHandler(messageHandler),
    m_inboundQueue(queue) {}

HiveMindBridgeImpl::~HiveMindBridgeImpl() {
    if (m_inboundThread.joinable()) {
        m_inboundThread.join();
    }
}

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

        for (auto result = m_resultQueue.begin(); result != m_resultQueue.end();) {
            if (result->getCallbackReturnContext().wait_for(std::chrono::seconds(0)) ==
                std::future_status::ready) {
                sendReturn(*result);

                m_resultQueue.erase(result);
            } else {
                result++;
            }
        }
    } else {
        if (m_inboundThread.joinable()) {
            m_inboundThread.join();
        }

        m_tcpServer.listen();

        if (greet()) {
            m_inboundThread = std::thread(&HiveMindBridgeImpl::inboundThread, this);
        } else {
            m_tcpServer.close();
        }
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

uint32_t HiveMindBridgeImpl::getSwarmAgentId() { return m_swarmAgentID; }

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
    std::optional<CallbackReturn> callbackReturnOpt = result.getCallbackReturnContext().get();

    // Send a return only if there is a return value
    if (callbackReturnOpt.has_value()) {
        CallbackArgs args = callbackReturnOpt.value().getReturnArgs();
        MessageDTO returnMessage = MessageUtils::createFunctionCallRequest(
            result.getMessageDestinationId(), // swap source and dest since we return to the sender
            result.getMessageSourceId(), MessageUtils::generateRandomId(),
            result.getSourceModule(), // swap source and dest since we return to the sender
            callbackReturnOpt.value().getReturnFunctionName(), args);

        m_serializer.serializeToStream(returnMessage);
    }
}

bool HiveMindBridgeImpl::greet() {
    if (isTCPClientConnected()) {
        MessageDTO message = MessageUtils::createGreetMessage();
        m_serializer.serializeToStream(message);

        MessageDTO greetResponse;
        m_deserializer.deserializeFromStream(greetResponse);

        std::optional<uint32_t> swarmAgentId = m_messageHandler.handleGreet(greetResponse);

        if (swarmAgentId.has_value()) {
            m_swarmAgentID = swarmAgentId.value();
            return true;
        }

        return false;
    }
}
