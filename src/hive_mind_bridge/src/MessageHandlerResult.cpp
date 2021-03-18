#include "hive_mind_bridge/MessageHandlerResult.h"

void MessageHandlerResult::setResponse(MessageDTO message) {
    m_responseMessage = message;
}

void MessageHandlerResult::setFuture(std::shared_future<std::optional<CallbackArgs>> future) {
    m_future = future;
}

MessageDTO MessageHandlerResult::getResponse() {
    return m_responseMessage;
}

std::shared_future<std::optional<CallbackArgs>> MessageHandlerResult::getFuture() {
    return m_future;
}