#ifndef HIVEMIND_BRIDGE_TCPSERVERINTERFACEMOCK_H
#define HIVEMIND_BRIDGE_TCPSERVERINTERFACEMOCK_H

#include "hiveboard_bridge/ITCPServer.h"
#include <gmock/gmock.h>

class TCPServerInterfaceMock : public ITCPServer {
  public:
    ~TCPServerInterfaceMock() = default;

    MOCK_METHOD(void, listen, ());
    MOCK_METHOD(bool, receive, (uint8_t * data, uint16_t length));
    MOCK_METHOD(bool, send, (const uint8_t* data, uint16_t length));
    MOCK_METHOD(void, close, ());
    MOCK_METHOD(void, onConnect, (std::function<void()> hook));
    MOCK_METHOD(void, onDisconnect, (std::function<void()> hook));
    MOCK_METHOD(bool, isClientConnected, (), (override));
};

#endif // HIVEMIND_BRIDGE_TCPSERVERINTERFACEMOCK_H
