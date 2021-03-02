#ifndef HIVEBOARD_BRIDGE_TCPSERVERINTERFACEMOCK_H
#define HIVEBOARD_BRIDGE_TCPSERVERINTERFACEMOCK_H

#include "hiveboard_bridge/ITCPServer.h"
#include <gmock/gmock.h>

class TCPServerInterfaceMock : public ITCPServer {
public:
    ~TCPServerInterfaceMock() = default;

    MOCK_METHOD(void, listen, ());
    MOCK_METHOD(bool, receive, (uint8_t* data, uint16_t length));
    MOCK_METHOD(bool, send, (const uint8_t* data, uint16_t length));
    MOCK_METHOD(void, close, ());
    MOCK_METHOD(void, onConnect, (Hook hook));
    MOCK_METHOD(void, onDisconnect, (Hook hook));
    MOCK_METHOD(bool, isClientConnected, (), (override));
};

#endif //HIVEBOARD_BRIDGE_TCPSERVERINTERFACEMOCK_H
