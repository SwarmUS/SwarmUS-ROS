#ifndef HIVEMIND_BRIDGE_HIVEMINDBRIDGEIMPL_H
#define HIVEMIND_BRIDGE_HIVEMINDBRIDGEIMPL_H

#include "hive_mind_bridge/IHiveMindBridge.h"
#include "hive_mind_bridge/MessageHandler.h"
#include "hive_mind_bridge/MessageHandlerResult.h"
#include "hive_mind_bridge/TCPServer.h"
#include "hive_mind_bridge/IThreadSafeQueue.h"
#include <deque>
#include <hivemind-host/HiveMindHostDeserializer.h>
#include <hivemind-host/HiveMindHostSerializer.h>
#include <memory>
#include <mutex>
#include <thread>

class HiveMindBridgeImpl : public IHiveMindBridge {
  public:
    /**
     * Construct a HiveMind Bridge object by injecting already-initialised objects.
     * @param tcpServer A TCPServer to be used
     * @param serializer A HiveMindHostSerializer to be used
     * @param deserializer A HiveMindHostDeserializer to be used
     */
    HiveMindBridgeImpl(ITCPServer& tcpServer,
                       IHiveMindHostSerializer& serializer,
                       IHiveMindHostDeserializer& deserializer,
                       IThreadSafeQueue<MessageDTO>& inboundQueue);

    void spin();

    void onConnect(std::function<void()> hook);

    void onDisconnect(std::function<void()> hook);

    bool registerCustomAction(std::string name,
                              CallbackFunction callback,
                              CallbackArgsManifest manifest);

    bool registerCustomAction(std::string name, CallbackFunction callback);

  private:
    ITCPServer& m_tcpServer;
    IHiveMindHostDeserializer& m_deserializer;
    IHiveMindHostSerializer& m_serializer;
    MessageHandler m_messageHandler;

    IThreadSafeQueue<MessageDTO>& m_inboundQueue;
    std::thread m_inboundThread;
    std::mutex m_mutex;

    std::deque<MessageHandlerResult> m_resultQueue;

    void inboundThread();
    bool isTCPClientConnected();
    void sendReturn(MessageHandlerResult result);
};

#endif // HIVEMIND_BRIDGE_HIVEMINDBRIDGEIMPL_H
