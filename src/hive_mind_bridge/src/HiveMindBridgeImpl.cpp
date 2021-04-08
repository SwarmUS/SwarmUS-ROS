#include "hive_mind_bridge/HiveMindBridgeImpl.h"

HiveMindBridgeImpl::HiveMindBridgeImpl(ITCPServer& tcpServer,
                                       IHiveMindHostSerializer& serializer,
                                       IHiveMindHostDeserializer& deserializer,
                                       IMessageHandler& messageHandler,
                                       IThreadSafeQueue<MessageDTO>& inboundQueue,
                                       IThreadSafeQueue<OutboundRequestHandle>& outboundQueue,
                                       ILogger& logger) :
    m_tcpServer(tcpServer),
    m_serializer(serializer),
    m_deserializer(deserializer),
    m_messageHandler(messageHandler),
    m_inboundQueue(inboundQueue),
    m_outboundQueue(outboundQueue),
    m_logger(logger) {}

HiveMindBridgeImpl::~HiveMindBridgeImpl() {
    if (m_inboundThread.joinable()) {
        m_inboundThread.join();
    }

    if (m_outboundThread.joinable()) {
        m_outboundThread.join();
    }
}

void HiveMindBridgeImpl::spin() {
    if (isTCPClientConnected()) {
        if (!m_inboundQueue.empty()) {
            auto vHandle = m_messageHandler.handleMessage(m_inboundQueue.front());
            m_inboundQueue.pop();

            if (std::holds_alternative<InboundRequestHandle>(vHandle)) {
                InboundRequestHandle handle = std::get<InboundRequestHandle>(vHandle);
                m_inboundRequestsQueue.push_back(handle);

                // Send the ack/nack message
                m_serializer.serializeToStream(handle.getResponse());
            } else if (std::holds_alternative<InboundResponseHandle>(vHandle)) {
                InboundResponseHandle handle = std::get<InboundResponseHandle>(vHandle);
                m_inboundResponsesMap[handle.getResponseId()] = handle;
            }
        }

        for (auto result = m_inboundRequestsQueue.begin();
             result != m_inboundRequestsQueue.end();) {
            if (result->getCallbackReturnContext().valid()) {
                if (result->getCallbackReturnContext().wait_for(std::chrono::seconds(0)) ==
                    std::future_status::ready) {
                    sendReturn(*result);

                    m_inboundRequestsQueue.erase(result);
                } else {
                    result++;
                }
            } else {
                m_inboundRequestsQueue.erase(result);
            }
        }
    } else {
        if (m_inboundThread.joinable()) {
            m_inboundThread.join();
        }

        if (m_outboundThread.joinable()) {
            m_outboundThread.join();
        }

        m_tcpServer.listen();

        if (greet()) {
            m_inboundThread = std::thread(&HiveMindBridgeImpl::inboundThread, this);
            m_outboundThread = std::thread(&HiveMindBridgeImpl::outboundThread, this);
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

bool HiveMindBridgeImpl::queueAndSend(MessageDTO message) {
    if (isTCPClientConnected()) {
        OutboundRequestHandle handle(message);
        m_outboundQueue.push(handle);
        return true;
    }

    return false;
}

uint32_t HiveMindBridgeImpl::getSwarmAgentId() { return m_swarmAgentID; }

void HiveMindBridgeImpl::inboundThread() {
    while (isTCPClientConnected()) {
        MessageDTO message;
        if (m_deserializer.deserializeFromStream(message)) {
            m_inboundQueue.push(message);
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(THREAD_SLEEP_MS));
}

void HiveMindBridgeImpl::outboundThread() {
    while (isTCPClientConnected()) {
        if (!m_outboundQueue.empty()) {
            OutboundRequestHandle handle = m_outboundQueue.front();

            MessageDTO outboundMessage = handle.getMessage();
            auto request = std::get_if<RequestDTO>(&outboundMessage.getMessage());

            if (request) {
                // verify if the front value has a corresponding inbound response handle
                auto search = m_inboundResponsesMap.find(request->getId());
                if (search != m_inboundResponsesMap.end()) {
                    // Received a response for this request so we delete the request
                    // TODO add some retry logic in case the response was not ok
                    m_outboundQueue.pop();
                    m_inboundResponsesMap.erase(search);
                    m_logger.log(LogLevel::Info, "RECEIVED VALID RESPONSE");
                } else {
                    // Did not receive a response for this request. Was the request sent?
                    if (handle.getState() == OutboundRequestState::READY) {
                        m_serializer.serializeToStream(outboundMessage);
                        handle.setState(OutboundRequestState::SENT);

                        m_outboundQueue.pop();
                        m_outboundQueue.push(handle); // Cycle through the queue
                    } else {
                        // Drop message or cycle through queue
                        if (handle.bumpDelaySinceSent(THREAD_SLEEP_MS) >= DELAY_BRFORE_DROP_S) {
                            // TODO add some retry logic after a timeout. For now, we simply drop.
                            m_outboundQueue.pop();
                        } else {
                            m_outboundQueue.pop();
                            m_outboundQueue.push(handle); // Cycle through the queue
                        }
                    }
                }
            } else {
                m_logger.log(LogLevel::Warn, "Outbound queue contains an unsupported message");
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(THREAD_SLEEP_MS));
    }
}

bool HiveMindBridgeImpl::isTCPClientConnected() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_tcpServer.isClientConnected();
}

void HiveMindBridgeImpl::sendReturn(InboundRequestHandle result) {
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
    }

    return false;
}
