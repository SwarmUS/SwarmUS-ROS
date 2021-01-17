#ifndef CATKIN_ROS_TCPSERVER_H
#define CATKIN_ROS_TCPSERVER_H

#include "ITCPServerMonitor.h"
#include "ros/ros.h"
#include <cstring>
#include <netinet/in.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>

#define TCP_BUFFER_LENGTH 65536 // max value of uint16

class TCPServer {
  public:
    /**
     * Construct a TCPServer
     * @param port Port to open the server to
     * @param monitor A monitor object to orchestrate the socket
     */
    TCPServer(int port, ITCPServerMonitor& monitor);
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
     * @param blocking Whether the call should block until some data is available.
     * @return The length of the data read.
     */
    int read(char* buff, const uint16_t length, bool blocking);

    /**
     * Send some data to the client.
     * @param buff The buffer containing the data to send.
     * @param length The length of the data on the buffer.
     */
    void send(char* buff, const uint16_t length);

    /**
     * Terminate a client connection.
     */
    void close();

  private:
    int m_serverFd, m_clientFd, m_port;
    int m_addressLength;
    struct sockaddr_in m_address;
    char m_buffer[TCP_BUFFER_LENGTH];

    ITCPServerMonitor& m_monitor;

    /**
     * Create and bind the socket.
     * @return
     */
    void init();
};

#endif // CATKIN_ROS_TCPSERVER_H
