#ifndef CATKIN_ROS_TCPSOCKET_H
#define CATKIN_ROS_TCPSOCKET_H

#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>

class TCPSocket {
public:
    /**
     *
     * @param port
     */
    TCPSocket(int port);

    /**
     * Create and bind the socket.
     * @return
     */
    int init();

private:
    int m_serverFd;
    int m_port;
    struct sockaddr_in m_address;
    char m_buffer[1024];
};


#endif //CATKIN_ROS_TCPSOCKET_H
