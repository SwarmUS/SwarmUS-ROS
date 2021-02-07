#ifndef HIVEBOARD_BRIDGE_RECEIVEACTION_H
#define HIVEBOARD_BRIDGE_RECEIVEACTION_H

#include "IMessageHandler.h"
#include <hivemind-host/IHiveMindHostDeserializer.h>

/**
 * @brief A class that contains all the actions that must be done in a receive thread.
 */
class ReceiveAction {
  public:
    ReceiveAction(IHiveMindHostDeserializer& deserializer, IMessageHandler& messageHandler);

    /**
     * Perform the actions.
     */
    void doAction();

  private:
    IHiveMindHostDeserializer& m_deserializer;
    IMessageHandler& m_messageHandler;
};

#endif // HIVEBOARD_BRIDGE_RECEIVEACTION_H
