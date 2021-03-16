#include "hive_mind_bridge/MessageHandlerResult.h"

void MessageHandlerResult::setResponse(MessageDTO message) {
    m_responseMessage = message;
}

void MessageHandlerResult::setReturnValues(std::future<std::optional<CallbackArgs>> returnValues) {
    m_returnValues = std::move(returnValues);
}

MessageDTO MessageHandlerResult::getResponse() {
    return m_responseMessage;
}

std::future<std::optional<CallbackArgs>> MessageHandlerResult::getReturnValues() {
    return std::move(m_returnValues);
}