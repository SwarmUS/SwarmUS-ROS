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

    void setMessageSourceId(uint32_t id);

    void setMessageDestinationId(uint32_t id);

    void setSourceModule(UserCallTargetDTO target);

    MessageDTO getResponse();

    std::shared_future<std::optional<CallbackArgs>> getFuture();

    std::string getReturnCallbackName();

    uint32_t getMessageSourceId();

    uint32_t getMessageDestinationId();

    UserCallTargetDTO getSourceModule();

private:
    std::string m_callbackName;
    MessageDTO m_responseMessage; // The acknowledge message to send as soon as possible
    std::shared_future<std::optional<CallbackArgs>> m_future;

    uint32_t m_msgSourceId;
    uint32_t m_msgDestinationId;
    UserCallTargetDTO m_sourceModule;
};

#endif //HIVE_MIND_BRIDGE_MESSAGEHANDLERRESULT_H
