#ifndef CATKIN_ROS_TCPSOCKET_H
#define CATKIN_ROS_TCPSOCKET_H

#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <cstring>
#include "ros/ros.h"    // TODO remove this

#define TCP_BUFFER_LENGTH 1024 // make this configurable?

class TCPSocket {
public:

    /**
     *
     * @param port
     */
    TCPSocket(int port);

    /**
     * Start listening for incoming client connections.
     * @param backlog
     * @return
     */
    int listen(int backlog);

    int accept();

    /**
     * Read data from socket.
     * @return
     */
    int read(const int length, char* buff); // const here?

    int send(const int length, char *buff); // const here?

    void loop();

private:
    int m_serverFd, m_clientFd, m_port, m_maxFd;
    int m_addressLength;
    struct sockaddr_in m_address;
    char m_buffer[TCP_BUFFER_LENGTH];
    fd_set m_fileDescriptorSet;

    /**
     * Create and bind the socket.
     * @return
     */
    int init();

};


#endif //CATKIN_ROS_TCPSOCKET_H
