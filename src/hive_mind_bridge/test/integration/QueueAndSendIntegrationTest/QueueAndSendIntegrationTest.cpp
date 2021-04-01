#include "ros/ros.h"
#include <chrono>
#include <hivemind-host/HiveMindHostDeserializer.h>
#include <hivemind-host/HiveMindHostSerializer.h>
#include <memory>
#include <string.h>
#include <thread>
#include "../utils/TCPClient.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "hive_mind_bridge_tester");

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // Create a TCP socket client
    TCPClient tcpClient;
    tcpClient.connect();
    HiveMindHostSerializer serializer(tcpClient);
    HiveMindHostDeserializer deserializer(tcpClient);

    // Wait for a greet message
    MessageDTO greetRequest;
    deserializer.deserializeFromStream(greetRequest);

    MessageDTO greetResponse(42, 42, GreetingDTO(42));
    serializer.serializeToStream(greetResponse);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Listen for request
    MessageDTO message;
    deserializer.deserializeFromStream(message);
    RequestDTO request = std::get<RequestDTO>(message.getMessage());
    UserCallRequestDTO userCallRequest = std::get<UserCallRequestDTO>(request.getRequest());
    FunctionCallRequestDTO functionCallRequest = std::get<FunctionCallRequestDTO>(userCallRequest.getRequest());
    std::string functionName = functionCallRequest.getFunctionName();

    ROS_INFO("REQUEST FROM HOST (%s): \n", functionName.c_str());

    // Send ack
    GenericResponseDTO genericResponse(GenericResponseStatusDTO::Ok, "");
    ResponseDTO response(request.getId(), genericResponse);
    MessageDTO ackMessage(message.getDestinationId(), message.getSourceId(), response);
    serializer.serializeToStream(ackMessage);

    ROS_INFO("SENT ACK");

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
}