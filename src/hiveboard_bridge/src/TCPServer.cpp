#include "hiveboard_bridge/TCPServer.h"

TCPServer::TCPServer(int port) {
    m_port = port;

    init();
}

TCPServer::~TCPServer() {
    close();
    ::close(m_serverFd);
}

void TCPServer::init() {
    int serverFd = 0;

    if ((serverFd = ::socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        ROS_ERROR("TCP server socket creation failed");
    }
    m_serverFd = serverFd;

    ROS_INFO("server FD: %d", m_serverFd);

    m_address.sin_family = AF_INET;
    m_address.sin_addr.s_addr = INADDR_ANY;
    m_address.sin_port = htons(m_port);

    m_addressLength = sizeof(m_address);

    if (::bind(m_serverFd, (struct sockaddr*)&m_address, m_addressLength) < 0) {
        ROS_ERROR("TCP server binding failed");
    }

    m_monitor.addServer(m_serverFd);
}

void TCPServer::listen() {
    if (::listen(m_serverFd, 1) < 0) {
        ROS_ERROR("TCP server listen failed");
    }

    if ((m_clientFd = ::accept(m_serverFd, (struct sockaddr*)&m_address,
                               (socklen_t*)&m_addressLength)) < 0) {
        ROS_ERROR("TCP server accept failed");
    }

    m_monitor.addConn(m_clientFd);
}

bool TCPServer::receive(uint8_t* data, uint16_t length) {
    int nbBytesReceived = ::recv(m_clientFd, data, length, MSG_WAITALL);
    ROS_INFO("RECIEVED : %s", (char*)data);

    if (nbBytesReceived == 0) { // Client disconnected
        ROS_WARN("TCP client disconnected");
        close();
        return false;
    }

    if (nbBytesReceived == -1)
        return false;

    return true;
}

bool TCPServer::send(const uint8_t* data, uint16_t length) {
    if (::send(m_clientFd, data, length, 0) > -1)
        return true;
    else
        return false;
}

void TCPServer::close() {
    ::close(m_clientFd);
    m_monitor.removeConn(m_clientFd);

    ROS_WARN("TCP server: client disconnected");
}
