#ifndef CATKIN_ROS_TCPSERVER_H
#define CATKIN_ROS_TCPSERVER_H

#include "ITCPServerMonitor.h"
#include "TCPServerMonitor.h"
#include "ros/ros.h"
#include <common/IProtobufStream.h>
#include <cstdint>
#include <cstring>
#include <memory>
#include <netinet/in.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>

class TCPServer : public IProtobufStream {
  public:
    /**
     * Construct a TCPServer
     * @param port Port to open the server to
     */
    TCPServer(int port);
    ~TCPServer();

    /**
     * Start listening for incoming client connections and accept one client. This is blocking until
     * a client connects.
     */
    void listen();

    /**
     * Read data from socket.
     * @param buff The buffer on which the incoming data will be written.
     * @param length The length of the provided buffer.
     * @return The length of the data read.
     */
    bool receive(uint8_t* data, uint16_t length) override;

    /**
     * Send some data to the client.
     * @param buff The buffer containing the data to send.
     * @param length The length of the data on the buffer.
     */
    bool send(const uint8_t* data, uint16_t length) override;

    /**
     * Terminate a client connection.
     */
    void close();

  private:
    int m_serverFd, m_clientFd, m_port;
    int m_addressLength;
    struct sockaddr_in m_address;
    TCPServerMonitor m_monitor;

    /**
     * Create and bind the socket.
     */
    void init();
};

#endif // CATKIN_ROS_TCPSERVER_H
