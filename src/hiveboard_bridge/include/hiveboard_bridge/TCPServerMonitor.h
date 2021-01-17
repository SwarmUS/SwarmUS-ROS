#ifndef CATKIN_ROS_TCPSERVERMONITOR_H
#define CATKIN_ROS_TCPSERVERMONITOR_H

#include "ITCPServerMonitor.h"
#include <cstring>

class TCPServerMonitor : public ITCPServerMonitor {
  public:
    TCPServerMonitor();
    ~TCPServerMonitor() override;

    void addServer(int serverFd) override;

    void addConn(int conn) override;

    void removeConn(int conn) override;

    bool waitIncoming(int conn) override;

  private:
    int m_serverFd;
    int m_maxFd;
    fd_set m_fileDescriptorSet;
};

#endif // CATKIN_ROS_TCPSERVERMONITOR_H
