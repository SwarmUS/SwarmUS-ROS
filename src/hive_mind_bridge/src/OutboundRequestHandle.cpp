#include "hive_mind_bridge/OutboundRequestHandle.h"

OutboundRequestHandle::OutboundRequestHandle(MessageDTO message) : m_message(message) {}

OutboundRequestState OutboundRequestHandle::getState() const { return m_state; }

void OutboundRequestHandle::setState(OutboundRequestState mState) { m_state = mState; }

const MessageDTO& OutboundRequestHandle::getMessage() const { return m_message; }

void OutboundRequestHandle::setMessage(const MessageDTO& mMessage) { m_message = mMessage; }

int OutboundRequestHandle::bumpDelaySinceSent(int bumpValue) {
    m_delaySinceSent += bumpValue;
    return m_delaySinceSent;
}