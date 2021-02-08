#ifndef HIVEBOARD_BRIDGE_RECEIVEACTION_H
#define HIVEBOARD_BRIDGE_RECEIVEACTION_H

#include "IMessageHandler.h"
#include <hivemind-host/IHiveMindHostDeserializer.h>

/**
 * @brief A class that contains all the actions that must be done to fetch and process a message.
 */
class ReceiveAction {
  public:
    ReceiveAction(IHiveMindHostDeserializer& deserializer, IMessageHandler& messageHandler);

    /**
     * Perform the actions.
     */
    void fetchAndProcessMessage();

  private:
    IHiveMindHostDeserializer& m_deserializer;
    IMessageHandler& m_messageHandler;
};

#endif // HIVEBOARD_BRIDGE_RECEIVEACTION_H
