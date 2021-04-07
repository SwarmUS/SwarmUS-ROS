#ifndef HIVEMIND_BRIDGE_ITCPSERVER_H
#define HIVEMIND_BRIDGE_ITCPSERVER_H

#include <functional>
#include <pheromones/IProtobufStream.h>

class ITCPServer : public IProtobufStream {
  public:
    /**
     * Start listening for incoming client connections and accept one client. This is blocking until
     * a client connects.
     */
    virtual void listen() = 0;

    /**
     * Read data from socket.
     * @param buff The buffer on which the incoming data will be written.
     * @param length The length of the provided buffer.
     * @return The length of the data read.
     */
    virtual bool receive(uint8_t* data, uint16_t length) = 0;

    /**
     * Send some data to the client.
     * @param buff The buffer containing the data to send.
     * @param length The length of the data on the buffer.
     */
    virtual bool send(const uint8_t* data, uint16_t length) = 0;

    /**
     * Terminate a client connection.
     */
    virtual void close() = 0;

    /**
     * See if the server has a client connected
     * @return true if the TCP server has a client connected.
     */
    virtual bool isClientConnected() = 0;

    /**
     * Register a callback to be run when a TCP connection is established with a client HiveMind
     * @param callback The function to be run
     */
    virtual void onConnect(std::function<void()> hook) = 0;

    /**
     * Register a callback to be run as soon as the TCP Server notices that the connection was lost.
     * @param callback The function to be run
     */
    virtual void onDisconnect(std::function<void()> hook) = 0;
};

#endif // HIVEMIND_BRIDGE_ITCPSERVER_H
