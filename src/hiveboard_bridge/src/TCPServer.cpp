#include "hiveboard_bridge/TCPServer.h"

TCPServer::TCPServer(int port) {
    m_port = port;
    bzero(m_buffer, TCP_BUFFER_LENGTH);

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

int TCPServer::read(char* buff, const uint16_t length, bool blocking) {
    if (blocking) {
        if (!m_monitor.waitIncoming(m_clientFd)) {
            ROS_ERROR("TCP server error waiting for incoming data");
        }
    }

    int nbBytesRecieved = ::recv(m_clientFd, m_buffer, length, 0);

    if (blocking && nbBytesRecieved == 0) { // Client disconnected
        close();
    }

    std::memcpy(buff, m_buffer, length);

    return nbBytesRecieved;
}

void TCPServer::send(char* buff, const uint16_t length) { ::send(m_clientFd, buff, length, 0); }

void TCPServer::close() {
    ::close(m_clientFd);
    m_monitor.removeConn(m_clientFd);

    ROS_WARN("TCP server: client disconnected");
}
