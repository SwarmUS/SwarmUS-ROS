#include "hiveboard_bridge/HiveBoardBridgeFactory.h"

HiveBoardBridge HiveBoardBridgeFactory::createHiveBoardBridge(int port) {
    TCPServer tcpServer(port);
    HiveMindHostSerializer serializer(tcpServer);
    HiveMindHostDeserializer deserializer(tcpServer);
    return HiveBoardBridge(tcpServer, serializer, deserializer);
}
