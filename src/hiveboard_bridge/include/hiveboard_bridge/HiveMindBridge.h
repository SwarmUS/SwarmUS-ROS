#ifndef HIVEBOARD_BRIDGE_HIVEBOARDBRIDGE_H
#define HIVEBOARD_BRIDGE_HIVEBOARDBRIDGE_H
#include "hiveboard_bridge/HiveMindBridgeImpl.h"
#include "hiveboard_bridge/IHiveMindBridge.h"
#include "hiveboard_bridge/MessageHandler.h"
#include "hiveboard_bridge/TCPServer.h"
#include <hivemind-host/HiveMindHostDeserializer.h>
#include <hivemind-host/HiveMindHostSerializer.h>
#include <memory>

class HiveBoardBridge : public IHiveBoardBridge {
  public:
    /**
     * Construct a HiveBoardBridge
     * @param tcpPort The port that the TCP server should listen to
     */
    HiveBoardBridge(int tcpPort);

    void spin();

    void onConnect(std::function<void()> hook);

    void onDisconnect(std::function<void()> hook);

    bool registerCustomAction(std::string name, CallbackFunction callback);

  private:
    std::unique_ptr<HiveBoardBridgeImpl> m_bridge;
    TCPServer m_tcpServer;
    HiveMindHostDeserializer m_deserializer;
    HiveMindHostSerializer m_serializer;
};

#endif // HIVEBOARD_BRIDGE_HIVEBOARDBRIDGE_H
