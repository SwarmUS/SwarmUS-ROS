#ifndef HIVEMIND_BRIDGE_MESSAGEHANDLERINTERFACEMOCK_H
#define HIVEMIND_BRIDGE_MESSAGEHANDLERINTERFACEMOCK_H

#include "hive_mind_bridge/IMessageHandler.h"
#include <gmock/gmock.h>
#include <hivemind-host/MessageDTO.h>
#include <variant>

class MessageHandlerInterfaceMock : public IMessageHandler {
  public:
    ~MessageHandlerInterfaceMock() = default;

    MOCK_METHOD((std::variant<std::monostate, InboundRequestHandle, InboundResponseHandle>), handleMessage, (MessageDTO message), (override));

    MOCK_METHOD(bool, registerCallback, (std::string name, CallbackFunction callback), (override));

    MOCK_METHOD(bool,
                registerCallback,
                (std::string name, CallbackFunction callback, CallbackArgsManifest manifest),
                (override));

    MOCK_METHOD(std::optional<CallbackFunction>,
                getCallback,
                (const std::string& name),
                (override));

    MOCK_METHOD(std::optional<uint32_t>, handleGreet, (MessageDTO message), (override));
};

#endif // HIVEMIND_BRIDGE_MESSAGEHANDLERINTERFACEMOCK_H
