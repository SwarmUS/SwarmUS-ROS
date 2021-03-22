#include "hive_mind_bridge/MessageHandlerResult.h"

void MessageHandlerResult::setResponse(MessageDTO message) { m_responseMessage = message; }

void MessageHandlerResult::setCallbackReturnContext(
    std::shared_future<std::optional<CallbackReturn>> future) {
    m_callbackReturnContext = future;
}

MessageDTO MessageHandlerResult::getResponse() { return m_responseMessage; }

std::shared_future<std::optional<CallbackReturn>> MessageHandlerResult::getCallbackReturnContext() {
    return m_callbackReturnContext;
}

void MessageHandlerResult::setCallbackName(std::string name) { m_callbackName = name; }

void MessageHandlerResult::setMessageSourceId(uint32_t id) { m_msgSourceId = id; }

void MessageHandlerResult::setMessageDestinationId(uint32_t id) { m_msgDestinationId = id; }

void MessageHandlerResult::setSourceModule(UserCallTargetDTO target) { m_sourceModule = target; }

uint32_t MessageHandlerResult::getMessageSourceId() { return m_msgSourceId; }

uint32_t MessageHandlerResult::getMessageDestinationId() { return m_msgDestinationId; }

UserCallTargetDTO MessageHandlerResult::getSourceModule() { return m_sourceModule; }