#include "hiveboard_bridge/StreamListener.h"

StreamListener::StreamListener(ReceiveThreadAction receiveThreadAction) :
    m_receiveThreadAction(receiveThreadAction) {
    m_rcvThread = std::thread(&StreamListener::receiveThread, this);
}

StreamListener::~StreamListener() {
    if (m_rcvThread.joinable()) {
        m_rcvThread.join();
    }
}

void StreamListener::receiveThread() {
    while (true) {
        m_receiveThreadAction.doAction();
    }
}
