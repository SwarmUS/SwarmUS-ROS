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

  private:
    OutboundRequestState m_state = OutboundRequestState::READY;
    MessageDTO m_message;
};

#endif // HIVE_MIND_BRIDGE_OUTBOUNDREQUESTHANDLE_H
