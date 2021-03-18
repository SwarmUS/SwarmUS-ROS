#ifndef HIVE_MIND_BRIDGE_MESSAGEHANDLERRESULT_H
#define HIVE_MIND_BRIDGE_MESSAGEHANDLERRESULT_H

#include <hivemind-host/MessageDTO.h>
#include <future>
#include <optional>
#include "UserCallbackFunctionWrapper.h"

class MessageHandlerResult {
public:

    MessageHandlerResult() = default;

    void setResponse(MessageDTO message);

    void setFuture(std::shared_future<std::optional<CallbackArgs>> future);

    MessageDTO getResponse();

    std::shared_future<std::optional<CallbackArgs>> getFuture();

private:
    MessageDTO m_responseMessage;
    std::shared_future<std::optional<CallbackArgs>> m_future;
};

#endif //HIVE_MIND_BRIDGE_MESSAGEHANDLERRESULT_H
