#ifndef HIVEBOARD_BRIDGE_MESSAGEHANDLERINTERFACEMOCK_H
#define HIVEBOARD_BRIDGE_MESSAGEHANDLERINTERFACEMOCK_H

#include "hiveboard_bridge/IMessageHandler.h"
#include <hivemind-host/MessageDTO.h>
#include <gmock/gmock.h>

class MessageHandlerInterfaceMock : public IMessageHandler {
public:
    ~MessageHandlerInterfaceMock() = default;

    MOCK_METHOD(bool, handleMessage, (MessageDTO message), (override));
};

#endif //HIVEBOARD_BRIDGE_MESSAGEHANDLERINTERFACEMOCK_H
