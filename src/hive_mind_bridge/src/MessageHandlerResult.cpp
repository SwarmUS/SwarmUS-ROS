#include "hive_mind_bridge/MessageHandlerResult.h"

void MessageHandlerResult::setResponse(MessageDTO message) {
    m_responseMessage = message;
}

void MessageHandlerResult::setReturnValues(std::shared_future<std::optional<CallbackArgs>> returnValues) {
//    m_returnValues = std::move(returnValues);
    m_returnValues = returnValues;
}

MessageDTO MessageHandlerResult::getResponse() {
    return m_responseMessage;
}

std::shared_future<std::optional<CallbackArgs>> MessageHandlerResult::getReturnValues() {
//    return std::move(m_returnValues);
    return m_returnValues;
}