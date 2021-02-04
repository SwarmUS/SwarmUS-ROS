#include "IMessageHandler.h"
#include <hivemind-host/IHiveMindHostDeserializer.h>

#ifndef HIVEBOARD_BRIDGE_RECEIVETHREADACTION_H
#define HIVEBOARD_BRIDGE_RECEIVETHREADACTION_H

/**
 * @brief A class that contains all the actions that must be done in a receive thread.
 */
class ReceiveThreadAction {
  public:
    ReceiveThreadAction(IHiveMindHostDeserializer& deserializer, IMessageHandler& messageHandler);

    /**
     * Perform the actions.
     */
    void doAction();

  private:
    IHiveMindHostDeserializer& m_deserializer;
    IMessageHandler& m_messageHandler;
};

#endif // HIVEBOARD_BRIDGE_RECEIVETHREADACTION_H
