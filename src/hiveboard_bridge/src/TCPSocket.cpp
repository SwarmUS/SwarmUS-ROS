#include "TCPSocket.h"

TCPSocket::TCPSocket(int port) {
    this->m_port = port;
    FD_ZERO(&m_fileDescriptorSet);
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

    if ((m_socket = ::accept(m_serverFd, (struct sockaddr*) &m_address, (socklen_t*) &m_addressLength)) < 0) {
        // ERROR
        return 0;
    }
}

int TCPSocket::read(const int length, char* buff) {
    if (length > TCP_BUFFER_LENGTH) {
        // ERROR
    }

    ::read(m_socket, m_buffer, length);
    std::memcpy(buff, m_buffer, length);

    return 0;
}

int TCPSocket::send(const int length, char *buff) {
    if (length > TCP_BUFFER_LENGTH) {
        // ERROR
    }

    ::send(m_socket, buff, length, 0);
}

void TCPSocket::loop() {
    ROS_INFO("Loop");

    fd_set tempFileDescriptorSet = m_fileDescriptorSet;

    int sel = select(m_serverFd + 1, &tempFileDescriptorSet, NULL, NULL, NULL);
    if(sel < 0) {
        //ERROR
    }

    char temp[12] = "";
    if (FD_ISSET(m_serverFd, &m_fileDescriptorSet)) {
        // EXECUTE CALLBACK FUNCTION
        read(12, temp);
        ROS_INFO("RECIEVED FROM CLIENT: %s", temp);

        FD_CLR(m_serverFd, &m_fileDescriptorSet);
    }
}