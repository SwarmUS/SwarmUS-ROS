#include "hive_mind_bridge/TCPServer.h"

TCPServer::TCPServer(int port, ILogger& logger) : m_logger(logger) {
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
}

void TCPServer::listen() {
    ROS_INFO("Listening for incoming clients on port %d...", m_port);

    if (::listen(m_serverFd, 1) < 0) {
        ROS_ERROR("TCP server listen failed");
    }

    if ((m_clientFd = ::accept(m_serverFd, (struct sockaddr*)&m_address,
                               (socklen_t*)&m_addressLength)) < 0) {
        ROS_ERROR("TCP server accept failed");
        m_isClientConnected = false;
    } else {
        if (m_onConnect) {
            m_onConnect();
        }
        m_isClientConnected = true;
    }
}

bool TCPServer::receive(uint8_t* data, uint16_t length) {
    int nbBytesReceived = ::recv(m_clientFd, data, length, MSG_WAITALL);

    if (nbBytesReceived == 0) { // Client disconnected
        ROS_WARN("TCP client disconnected");
        if (m_onDisonnect) {
            m_onDisonnect();
        }
        m_isClientConnected = false;
        close();
    }

    return nbBytesReceived == length;
}

bool TCPServer::send(const uint8_t* data, uint16_t length) {
    if (::send(m_clientFd, data, length, 0) > -1)
        return true;
    else
        return false;
}

void TCPServer::close() {
    ::close(m_clientFd);
    m_isClientConnected = false;
    ROS_WARN("TCP server: client disconnected");
}

bool TCPServer::isClientConnected() { return m_isClientConnected; }

void TCPServer::onConnect(std::function<void()> hook) { m_onConnect = hook; }

void TCPServer::onDisconnect(std::function<void()> hook) { m_onDisonnect = hook; }