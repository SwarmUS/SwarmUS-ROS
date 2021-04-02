#ifndef HIVE_MIND_BRIDGE_TCPCLIENT_H
#define HIVE_MIND_BRIDGE_TCPCLIENT_H

#include <arpa/inet.h>
#include <common/IProtobufStream.h>
#include <cstdio>
#include <netinet/in.h>
#include <sys/socket.h>
#include <cstdio>

class TCPClient : public IProtobufStream {
  public:
    TCPClient(int port) : m_port(port) { m_clientFd = ::socket(AF_INET, SOCK_STREAM, 0); }

    int connect() {
        struct sockaddr_in serv_addr;

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(m_port);

        // Convert IPv4 and IPv6 addresses from text to binary form
        if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
            printf("\nInvalid address/ Address not supported \n");
            return -1;
        }

        ::connect(m_clientFd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
    }

    bool receive(uint8_t* data, uint16_t length) override {
        int nbBytesReceived = ::recv(m_clientFd, data, length, MSG_WAITALL);

        return nbBytesReceived == length;
    }

    bool send(const uint8_t* data, uint16_t length) override {
        if (::send(m_clientFd, data, length, 0) > -1)
            return true;
        else
            return false;
    }

  private:
    int m_clientFd;
    int m_port;
};

#endif // HIVE_MIND_BRIDGE_TCPCLIENT_H
