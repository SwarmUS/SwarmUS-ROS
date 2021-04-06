#ifndef HIVE_MIND_BRIDGE_OUTBOUNDREQUESTHANDLE_H
#define HIVE_MIND_BRIDGE_OUTBOUNDREQUESTHANDLE_H

#include <hivemind-host/MessageDTO.h>

enum class OutboundRequestState { READY, SENT };

class OutboundRequestHandle {
  public:
    OutboundRequestHandle(MessageDTO message);

    OutboundRequestState getState() const;

    void setState(OutboundRequestState mState);

    const MessageDTO& getMessage() const;

    void setMessage(const MessageDTO& mMessage);

    /**
     * Increment the delay since sent
     * @param bumpValue Value to add to the delay
     * @return The delay since sent, incremented with bumpValue
     */
    int bumpDelaySinceSent(int bumpValue);

  private:
    OutboundRequestState m_state = OutboundRequestState::READY;
    MessageDTO m_message;
    int m_delaySinceSent = 0;
};

#endif // HIVE_MIND_BRIDGE_OUTBOUNDREQUESTHANDLE_H
