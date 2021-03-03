#ifndef HIVEBOARD_BRIDGE_IHIVEBOARDBRIDGE_H
#define HIVEBOARD_BRIDGE_IHIVEBOARDBRIDGE_H

#include "IMessageHandler.h"
#include <string.h>

/**
 * The HiveBoardBridge is the main entrypoint to the bridge's library. It includes everything
 * necessary to define a custom swarm API and provides some hooks to control the robot when some
 * usual events occur, such as connection or disconnection of a HiveBoard.
 */
class IHiveBoardBridge {
  public:
    /**
     * Spin the bridge's applicative loop
     */
    virtual void spin() = 0;

    /**
     * Register a callback to be run when a TCP connection is established with a client HiveBoard
     * @param callback The function to be run
     */
    virtual void onConnect(std::function<void()> hook) = 0;

    /**
     * Register a callback to be run as soon as the TCP Server notices that the connection was lost.
     * @param callback The function to be run
     */
    virtual void onDisconnect(std::function<void()> hook) = 0;

    /**
     * Register a custom action that this robot can accomplish.
     * @param name The name of the action (must match across the swarm components)
     * @param callback The function to be run. This is where the custom robot behaviour is meant to
     * be defined.
     * @return True if an existing callback function was overwritten, false otherwise
     */
    virtual bool registerCustomAction(std::string name, CallbackFunction callback) = 0;
};

#endif // HIVEBOARD_BRIDGE_IHIVEBOARDBRIDGE_H
