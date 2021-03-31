#ifndef HIVE_MIND_BRIDGE_INBOUNDREQUESTHANDLE_H
#define HIVE_MIND_BRIDGE_INBOUNDREQUESTHANDLE_H

#include "hive_mind_bridge/Callback.h"
#include "hive_mind_bridge/UserCallbackFunctionWrapper.h"
#include <future>
#include <hivemind-host/MessageDTO.h>
#include <optional>

/**
 * A class that contains various data to be returned by the handling of an incoming message.
 */
class InboundRequestHandle {
  public:
    InboundRequestHandle() = default;

    void setResponse(MessageDTO message);

    void setCallbackReturnContext(std::shared_future<std::optional<CallbackReturn>> future);

    void setMessageSourceId(uint32_t id);

    void setMessageDestinationId(uint32_t id);

    void setSourceModule(UserCallTargetDTO target);

    void setCallbackName(std::string name);

    MessageDTO getResponse();

    std::shared_future<std::optional<CallbackReturn>> getCallbackReturnContext();

    uint32_t getMessageSourceId();

    uint32_t getMessageDestinationId();

    UserCallTargetDTO getSourceModule();

  private:
    std::string m_callbackName; // The name of the callback called by a functionCallRequest
    MessageDTO m_responseMessage; // The acknowledge message to send as soon as possible
    std::shared_future<std::optional<CallbackReturn>> m_callbackReturnContext;

    uint32_t m_msgSourceId;
    uint32_t m_msgDestinationId;
    UserCallTargetDTO m_sourceModule;
};

#endif // HIVE_MIND_BRIDGE_INBOUNDREQUESTHANDLE_H
