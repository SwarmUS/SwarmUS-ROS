#ifndef HIVEMIND_BRIDGE_MESSAGEHANDLERINTERFACEMOCK_H
#define HIVEMIND_BRIDGE_MESSAGEHANDLERINTERFACEMOCK_H

#include "hive_mind_bridge/IMessageHandler.h"
#include <gmock/gmock.h>
#include <hivemind-host/MessageDTO.h>

class MessageHandlerInterfaceMock : public IMessageHandler {
  public:
    ~MessageHandlerInterfaceMock() = default;

    MOCK_METHOD(MessageHandlerResult, handleMessage, (MessageDTO message), (override));

    MOCK_METHOD(bool, registerCallback, (std::string name, CallbackFunction callback), (override));

    MOCK_METHOD(bool, registerCallback, (std::string name, CallbackFunction callback, CallbackArgsManifest manifest), (override));

    MOCK_METHOD(std::optional<CallbackFunction>,
                getCallback,
                (const std::string& name),
                (override));
};

#endif // HIVEMIND_BRIDGE_MESSAGEHANDLERINTERFACEMOCK_H
