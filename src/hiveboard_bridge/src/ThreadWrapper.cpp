#include "hiveboard_bridge/ThreadWrapper.h"

ThreadWrapper::ThreadWrapper(ReceiveAction receiveAction, int sleepTimeMs) :
    m_receiveAction(receiveAction), m_sleepTimeMs(sleepTimeMs) {
    m_rcvThread = std::thread(&ThreadWrapper::receiveThread, this);
}

ThreadWrapper::~ThreadWrapper() {
    if (m_rcvThread.joinable()) {
        m_threadShouldRun = false;
        m_rcvThread.join();
    }
}

void ThreadWrapper::receiveThread() {
    while (m_threadShouldRun) {
        m_receiveAction.fetchAndProcessMessage();
        std::this_thread::sleep_for(std::chrono::milliseconds(m_sleepTimeMs));
    }
}
