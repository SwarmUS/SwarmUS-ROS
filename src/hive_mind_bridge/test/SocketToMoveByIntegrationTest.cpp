#include "hive_mind_bridge/MessageUtils.h"
#include "ros/ros.h"
#include <arpa/inet.h>
#include <chrono>
#include <common/IProtobufStream.h>
#include <cstdint>
#include <hivemind-host/HiveMindHostDeserializer.h>
#include <hivemind-host/HiveMindHostSerializer.h>
#include <memory>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <thread>

class TCPClient : public IProtobufStream {
  public:
    TCPClient() { m_clientFd = ::socket(AF_INET, SOCK_STREAM, 0); }

    int connect() {
        struct sockaddr_in serv_addr;

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(5555);

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
};

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

    // Create a moveBy function call wrapped in a message
    FunctionCallArgumentDTO moveByX((float)1);
    FunctionCallArgumentDTO moveByY((float)1);
    FunctionCallArgumentDTO args[2] = {moveByX, moveByY};
    FunctionCallRequestDTO moveByFunctionCallRequestDTO("moveBy", args, 2);
    UserCallRequestDTO userCallRequest(UserCallTargetDTO::UNKNOWN, UserCallTargetDTO::HOST,
                                       moveByFunctionCallRequestDTO);
    RequestDTO moveByRequestDTO(1, userCallRequest);
    MessageDTO moveByMessageDTO(1, 2, moveByRequestDTO);

    MessageDTO getRobotStatusMessage =
        MessageUtils::createFunctionCallRequest(1, 2, 99, UserCallTargetDTO::HOST, "getStatus");

    // Send getStatus request
    serializer.serializeToStream(getRobotStatusMessage);

    // Listen for ack
    MessageDTO statusResponseMessage;
    deserializer.deserializeFromStream(statusResponseMessage);
    ResponseDTO statusResponse = std::get<ResponseDTO>(statusResponseMessage.getMessage());
    UserCallResponseDTO statusUserCallResponse =
        std::get<UserCallResponseDTO>(statusResponse.getResponse());
    FunctionCallResponseDTO statusFunctionCallResponse =
        std::get<FunctionCallResponseDTO>(statusUserCallResponse.getResponse());
    GenericResponseDTO statusGenericResponse = statusFunctionCallResponse.getResponse();
    GenericResponseStatusDTO statusStatus = statusGenericResponse.getStatus();
    std::string statusDetails = statusGenericResponse.getDetails();
    ROS_INFO("RESPONSE FROM HOST (getStatus): \n"
             "\tResponse status: %d\n"
             "\tDetails: %s",
             statusStatus, statusDetails.c_str());

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Lister for return
    MessageDTO statusReturnMessage;
    deserializer.deserializeFromStream(statusReturnMessage);
    RequestDTO statusReturnRequest = std::get<RequestDTO>(statusReturnMessage.getMessage());
    UserCallRequestDTO statusReturnUserCallRequest =
        std::get<UserCallRequestDTO>(statusReturnRequest.getRequest());
    FunctionCallRequestDTO statusReturnFunctionCallRequest =
        std::get<FunctionCallRequestDTO>(statusReturnUserCallRequest.getRequest());
    std::string statusReturnFunctionName = statusReturnFunctionCallRequest.getFunctionName();
    std::array statusReturnFunctionArgs = statusReturnFunctionCallRequest.getArguments();
    int64_t arg0 = std::get<int64_t>(statusReturnFunctionArgs[0].getArgument());

    ROS_INFO("RESPONSE FROM HOST (%s): \n"
             "\tPayload: %d",
             statusReturnFunctionName.c_str(), arg0);

    for (int i = 0; i < 5; i++) {
        // Send moveBy request
        serializer.serializeToStream(moveByMessageDTO);

        // Listen for a response
        MessageDTO message;
        deserializer.deserializeFromStream(message);
        ResponseDTO response = std::get<ResponseDTO>(message.getMessage());
        UserCallResponseDTO userCallResponse =
            std::get<UserCallResponseDTO>(response.getResponse());
        FunctionCallResponseDTO functionCallResponse =
            std::get<FunctionCallResponseDTO>(userCallResponse.getResponse());
        GenericResponseDTO genericResponse = functionCallResponse.getResponse();
        GenericResponseStatusDTO status = genericResponse.getStatus();
        std::string details = genericResponse.getDetails();
        ROS_INFO("RESPONSE FROM HOST (moveBy): \n"
                 "\tResponse status: %d\n"
                 "\tDetails: %s",
                 status, details.c_str());
    }
}