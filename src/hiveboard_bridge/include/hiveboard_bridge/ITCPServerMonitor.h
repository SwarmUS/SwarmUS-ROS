#ifndef CATKIN_ROS_ITCPSERVERMONITOR_H
#define CATKIN_ROS_ITCPSERVERMONITOR_H

#include <sys/select.h>

class ITCPServerMonitor {
  public:
    virtual ~ITCPServerMonitor() = default;

    /**
     * Add the server's file descriptor to the watch list.
     * @param serverFd File descriptor of the server.
     */
    virtual void addServer(int serverFd) = 0;

    /**
     * Add a connection to the watch list.
     * @param conn File descriptor of the connection.
     */
    virtual void addConn(int conn) = 0;

    /**
     * Remove a connection from the watch list.
     * @param conn File descriptor of the connection.
     */
    virtual void removeConn(int conn) = 0;

    /**
     * Wait for until there is incoming data on a connection.
     * @param conn File descriptor of the connection.
     * @return
     */
    virtual bool waitIncoming(int conn) = 0;
};

#endif // CATKIN_ROS_ITCPSERVERMONITOR_H
