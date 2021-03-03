#ifndef HIVEBOARD_BRIDGE_HIVEBOARDBRIDGEIMPL_H
#define HIVEBOARD_BRIDGE_HIVEBOARDBRIDGEIMPL_H

#include "hiveboard_bridge/IHiveMindBridge.h"
#include "hiveboard_bridge/MessageHandler.h"
#include "hiveboard_bridge/TCPServer.h"
#include <hivemind-host/HiveMindHostDeserializer.h>
#include <hivemind-host/HiveMindHostSerializer.h>
#include <memory>

class HiveBoardBridgeImpl : public IHiveBoardBridge {
  public:
    /**
     * Construct a HiveBoard Bridge object by injecting already-initialised objects.
     * @param tcpServer A TCPServer to be used
     * @param serializer A HiveMindHostSerializer to be used
     * @param deserializer A HiveMindHostDeserializer to be used
     */
    HiveBoardBridgeImpl(ITCPServer& tcpServer,
                        IHiveMindHostSerializer& serializer,
                        IHiveMindHostDeserializer& deserializer);

    void spin();

    void onConnect(std::function<void()> hook);

    void onDisconnect(std::function<void()> hook);

    bool registerCustomAction(std::string name, CallbackFunction callback);

  private:
    ITCPServer& m_tcpServer;
    IHiveMindHostDeserializer& m_deserializer;
    IHiveMindHostSerializer& m_serializer;
    MessageHandler m_messageHandler;
};

#endif // HIVEBOARD_BRIDGE_HIVEBOARDBRIDGEIMPL_H
