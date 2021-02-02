#include "hiveboard_bridge/StreamListener.h"

StreamListener::StreamListener(int port, MessageHandler messageHandler) :
m_socket(port),
m_messageHandler(messageHandler) {
    m_rcvThread = std::thread(&StreamListener::receiveThread, this);
}

StreamListener::~StreamListener() {
    if (m_rcvThread.joinable()) {
        m_rcvThread.join();
    }
}

void StreamListener::receiveThread() {
    // Initialisation of the socket
    ROS_INFO("Listening for incoming clients...");
    m_socket.listen();
    ROS_INFO("Client connected.");

    char sendValue[] = "Hello world from ROS!";
    m_socket.send(sendValue, strlen(sendValue));
    ROS_INFO("Sent a message from server to client");

    char buf[10] = "";
    char okBuf[3] = "Ok";
    int i = 0;

    while (true) {
        bzero(buf, 10);
        m_socket.read(buf, 10, true);
        ROS_INFO("Received from thread: %s", buf);
        buf[5] = '\0';
        std::string str(buf);
//        m_messageHandler.handleMessage(str);
    }
}