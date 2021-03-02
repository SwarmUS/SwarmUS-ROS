#ifndef HIVEBOARD_BRIDGE_HIVEBOARDBRIDGE_H
#define HIVEBOARD_BRIDGE_HIVEBOARDBRIDGE_H
#include "hiveboard_bridge/MessageHandler.h"
#include "hiveboard_bridge/TCPServer.h"
#include <hivemind-host/HiveMindHostDeserializer.h>
#include <hivemind-host/HiveMindHostSerializer.h>
#include <memory>

/**
 * The HiveBoardBridge class is the main entrypoint to the bridge's library. It includes everything necessary to
 * define a custom swarm API and provides some hooks to control the robot when some usual events occur,
 * such as connection or disconnection of a client.
 */
class HiveBoardBridge {
  public:
    /**
     * Construct a HiveBoard Bridge object by injecting already-initialised objects.
     * @param tcpServer A TCPServer to be used
     * @param serializer A HiveMindHostSerializer to be used
     * @param deserializer A HiveMindHostDeserializer to be used
     */
    HiveBoardBridge(ITCPServer& tcpServer, IHiveMindHostSerializer& serializer, IHiveMindHostDeserializer& deserializer);

    /**
     * Spin the bridge's applicative loop
     */
    void spin();

    /**
     * Register a callback to be run when a TCP connection is established with a client HiveBoard
     * @param callback The function to be run
     */
    void onConnect(Hook hook);

    /**
     * Register a callback to be run as soon as the TCP Server notices that the connection was lost.
     * @param callback The function to be run
     */
    void onDisconnect(Hook hook);

    /**
     * Register a custom action that this robot can accomplish.
     * @param name The name of the action (must match across the swarm components)
     * @param callback The function to be run. This is where the custom robot behaviour is meant to be defined.
     * @return True if an existing callback function was overwritten, false otherwise
     */
    bool registerCustomAction(std::string name, CallbackFunction callback);

  private:
    ITCPServer& m_tcpServer;
    IHiveMindHostDeserializer& m_deserializer;
    IHiveMindHostSerializer& m_serializer;
    MessageHandler m_messageHandler;
};

#endif // HIVEBOARD_BRIDGE_HIVEBOARDBRIDGE_H
