#ifndef HIVEBOARD_BRIDGE_HIVEBOARDBRIDGE_H
#define HIVEBOARD_BRIDGE_HIVEBOARDBRIDGE_H
#include "hiveboard_bridge/HiveBoardBridgeImpl.h"
#include "hiveboard_bridge/MessageHandler.h"
#include "hiveboard_bridge/TCPServer.h"
#include <hivemind-host/HiveMindHostDeserializer.h>
#include <hivemind-host/HiveMindHostSerializer.h>
#include <memory>

/**
 * The HiveBoardBridge class is the main entrypoint to the bridge's library. It includes everything
 * necessary to define a custom swarm API and provides some hooks to control the robot when some
 * usual events occur, such as connection or disconnection of a HiveBoard.
 */
class HiveBoardBridge {
  public:
    /**
     * Construct a HiveBoardBridge
     * @param tcpPort The port that the TCP server should listen to
     */
    HiveBoardBridge(int tcpPort);

    /**
     * Spin the bridge's applicative loop
     */
    void spin();

    /**
     * Register a callback to be run when a TCP connection is established with a client HiveBoard
     * @param callback The function to be run
     */
    void onConnect(std::function<void()> hook);

    /**
     * Register a callback to be run as soon as the TCP Server notices that the connection was lost.
     * @param callback The function to be run
     */
    void onDisconnect(std::function<void()> hook);

    /**
     * Register a custom action that this robot can accomplish.
     * @param name The name of the action (must match across the swarm components)
     * @param callback The function to be run. This is where the custom robot behaviour is meant to
     * be defined.
     * @return True if an existing callback function was overwritten, false otherwise
     */
    bool registerCustomAction(std::string name, CallbackFunction callback);

  private:
    std::unique_ptr<HiveBoardBridgeImpl> m_bridge;
    TCPServer m_tcpServer;
    HiveMindHostDeserializer m_deserializer;
    HiveMindHostSerializer m_serializer;
};

#endif // HIVEBOARD_BRIDGE_HIVEBOARDBRIDGE_H
