#ifndef HIVEMIND_BRIDGE_HIVEMINDBRIDGE_H
#define HIVEMIND_BRIDGE_HIVEMINDBRIDGE_H
#include "hive_mind_bridge/HiveMindBridgeImpl.h"
#include "hive_mind_bridge/IHiveMindBridge.h"
#include "hive_mind_bridge/MessageHandler.h"
#include "hive_mind_bridge/OutboundRequestHandle.h"
#include "hive_mind_bridge/TCPServer.h"
#include "hive_mind_bridge/ThreadSafeQueue.h"
#include <cpp-common/ILogger.h>
#include <memory>
#include <pheromones/HiveMindHostDeserializer.h>
#include <pheromones/HiveMindHostSerializer.h>

class HiveMindBridge : public IHiveMindBridge {
  public:
    /**
     * Construct a HiveMindBridge
     * @param tcpPort The port that the TCP server should listen to
     */
    HiveMindBridge(int tcpPort, ILogger& logger);

    ~HiveMindBridge();

    void spin();

    void onConnect(std::function<void()> hook);

    void onDisconnect(std::function<void()> hook);

    bool registerCustomAction(std::string name,
                              CallbackFunction callback,
                              CallbackArgsManifest manifest);

    bool registerCustomAction(std::string name, CallbackFunction callback);

    bool queueAndSend(MessageDTO message);

  private:
    ILogger& m_logger;
    std::unique_ptr<HiveMindBridgeImpl> m_bridge;
    TCPServer m_tcpServer;
    HiveMindHostDeserializer m_deserializer;
    HiveMindHostSerializer m_serializer;
    MessageHandler m_messageHandler;
    ThreadSafeQueue<MessageDTO> m_inboundQueue;
    ThreadSafeQueue<OutboundRequestHandle> m_outboundQueue;
};

#endif // HIVEMIND_BRIDGE_HIVEMINDBRIDGE_H
