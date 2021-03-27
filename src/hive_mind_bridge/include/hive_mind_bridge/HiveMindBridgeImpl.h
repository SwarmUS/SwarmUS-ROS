#ifndef HIVEMIND_BRIDGE_HIVEMINDBRIDGEIMPL_H
#define HIVEMIND_BRIDGE_HIVEMINDBRIDGEIMPL_H

#include "hive_mind_bridge/Callback.h"
#include "hive_mind_bridge/IHiveMindBridge.h"
#include "hive_mind_bridge/IThreadSafeQueue.h"
#include "hive_mind_bridge/MessageHandler.h"
#include "hive_mind_bridge/MessageHandlerResult.h"
#include "hive_mind_bridge/TCPServer.h"
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
     * @param messageHandler A MessageHandler to be used
     * @param deserializer A HiveMindHostDeserializer to be used
     * @param inboundQueue A ThreadSafeQueue to be  used
     */
    HiveMindBridgeImpl(ITCPServer& tcpServer,
                       IHiveMindHostSerializer& serializer,
                       IHiveMindHostDeserializer& deserializer,
                       IMessageHandler& messageHandler,
                       IThreadSafeQueue<MessageDTO>& inboundQueue);

    ~HiveMindBridgeImpl();

    void spin();

    void onConnect(std::function<void()> hook);

    void onDisconnect(std::function<void()> hook);

    bool registerCustomAction(std::string name,
                              CallbackFunction callback,
                              CallbackArgsManifest manifest);

    bool registerCustomAction(std::string name, CallbackFunction callback);

    uint32_t getSwarmId();

  private:
    ITCPServer& m_tcpServer;
    IHiveMindHostDeserializer& m_deserializer;
    IHiveMindHostSerializer& m_serializer;
    IMessageHandler& m_messageHandler;

    IThreadSafeQueue<MessageDTO>& m_inboundQueue;
    std::thread m_inboundThread;
    std::mutex m_mutex;

    std::deque<MessageHandlerResult> m_resultQueue;

    uint32_t m_swarmID = 0; // Maybe rename this

    void inboundThread();
    bool isTCPClientConnected();
    void sendReturn(MessageHandlerResult result);
    bool greet();
};

#endif // HIVEMIND_BRIDGE_HIVEMINDBRIDGEIMPL_H
