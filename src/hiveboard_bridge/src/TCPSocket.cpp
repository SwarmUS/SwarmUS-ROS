#include "TCPSocket.h"

TCPSocket::TCPSocket(int port) {
    this->m_port = port;
}

int TCPSocket::init() {
    int serverFd = 0;
    if ((serverFd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        // ERROR, print this to a logger
    }
    m_serverFd = serverFd;

    m_address.sin_family = AF_INET;
    m_address.sin_addr.s_addr = INADDR_ANY;
    m_address.sin_port = htons(m_port);

    if (bind(m_serverFd, (struct sockaddr*) &m_address, sizeof(m_address)) < 0) {
        // ERROR
    }

    listen(m_serverFd, 1);
}

