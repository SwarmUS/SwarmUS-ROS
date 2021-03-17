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

    void setReturnValues(std::shared_future<std::optional<CallbackArgs>> returnValues);

    MessageDTO getResponse();

    std::shared_future<std::optional<CallbackArgs>> getReturnValues();

private:
    MessageDTO m_responseMessage;
    std::shared_future<std::optional<CallbackArgs>> m_returnValues;
};

#endif //HIVE_MIND_BRIDGE_MESSAGEHANDLERRESULT_H
