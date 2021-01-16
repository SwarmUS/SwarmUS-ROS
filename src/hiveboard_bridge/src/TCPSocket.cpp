#include "TCPSocket.h"

TCPSocket::TCPSocket(int port) {
    this->m_port = port;
    m_maxFd = 0;
    FD_ZERO(&m_fileDescriptorSet);
    bzero(m_buffer, TCP_BUFFER_LENGTH);
    init();
}

int TCPSocket::init() {
    int ret;
    int serverFd = 0;

    ret = serverFd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (ret == 0) {
        // ERROR, print this to a logger
        return ret;
    }
    m_serverFd = serverFd;

    m_address.sin_family = AF_INET;
    m_address.sin_addr.s_addr = INADDR_ANY;
    m_address.sin_port = htons(m_port);

    m_addressLength = sizeof(m_address);

    ret = ::bind(m_serverFd, (struct sockaddr*) &m_address, m_addressLength);
    if (ret < 0) {
        // ERROR
        return ret;
    }

    // Add the socket to the file descriptor set
    FD_SET(m_serverFd, &m_fileDescriptorSet);
}

int TCPSocket::listen(int backlog) {
    int ret = ::listen(m_serverFd, backlog);

    if (ret < 0) {
        // ERROR
        return ret;
    }

    accept();
}

int TCPSocket::accept() {
    if ((m_clientFd = ::accept(m_serverFd, (struct sockaddr*) &m_address, (socklen_t*) &m_addressLength)) < 0) {
        // ERROR
        return 0;
    }

    FD_SET(m_clientFd, &m_fileDescriptorSet);

    if (m_clientFd > m_maxFd) {
        m_maxFd = m_clientFd;
    }

    send(3, "OK");
}

int TCPSocket::read(const int length, char* buff) {
    if (length > TCP_BUFFER_LENGTH) {
        // ERROR
    }

    int nbBytesRecieved = ::recv(m_clientFd, m_buffer, length, 0);
    if (nbBytesRecieved <= 0) {
        if (nbBytesRecieved == 0) { // disconnect
            ::close(m_clientFd);
            FD_CLR(m_clientFd, &m_fileDescriptorSet);
            m_maxFd = m_serverFd; // decrement the max since we closed a socket.
        }
    }

    std::memcpy(buff, m_buffer, length);

    return 0;
}

int TCPSocket::send(const int length, char *buff) {
    if (length > TCP_BUFFER_LENGTH) {
        // ERROR
    }

    ::send(m_clientFd, buff, length, 0);
}

void TCPSocket::loop() {
    fd_set tempFileDescriptorSet = m_fileDescriptorSet;

    int sel = select(m_maxFd + 1, &tempFileDescriptorSet, NULL, NULL, NULL);
    if(sel < 0) {
        //ERROR
    }

    char temp[12] = ""; // TODO remove this

    for (int i = 0; i <= m_maxFd; i++) {
        if (FD_ISSET(i, &tempFileDescriptorSet)) {
            if (i == m_serverFd) { // There is new activity on the server FD
                accept();
            } else { // The remaining FDs are the sockets that have some data ready
                // TODO EXECUTE CALLBACK FUNCTION
                read(12, temp);
                ROS_INFO("RECIEVED FROM CLIENT: %s", temp);
                bzero(m_buffer, TCP_BUFFER_LENGTH);
            }
        }
    }
}