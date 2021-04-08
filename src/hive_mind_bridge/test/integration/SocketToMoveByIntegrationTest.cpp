#include "../utils/Logger.h"
#include "../utils/TCPClient.h"
#include "hive_mind_bridge/MessageUtils.h"
#include <chrono>
#include <cstdint>
#include <pheromones/HiveMindHostDeserializer.h>
#include <pheromones/HiveMindHostSerializer.h>
#include <thread>

int main(int argc, char** argv) {
    Logger logger;

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // Create a TCP socket client
    TCPClient tcpClient(5555);
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
    logger.log(LogLevel::Info,
               "RESPONSE FROM HOST (getStatus): \n"
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

    logger.log(LogLevel::Info,
               "RESPONSE FROM HOST (%s): \n"
               "\tPayload: %d",
               statusReturnFunctionName.c_str(), arg0);

    // Send moveBy request
    serializer.serializeToStream(moveByMessageDTO);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Listen for a response
    MessageDTO message;
    deserializer.deserializeFromStream(message);
    ResponseDTO response = std::get<ResponseDTO>(message.getMessage());
    UserCallResponseDTO userCallResponse = std::get<UserCallResponseDTO>(response.getResponse());
    FunctionCallResponseDTO functionCallResponse =
        std::get<FunctionCallResponseDTO>(userCallResponse.getResponse());
    GenericResponseDTO genericResponse = functionCallResponse.getResponse();
    GenericResponseStatusDTO status = genericResponse.getStatus();
    std::string details = genericResponse.getDetails();
    logger.log(LogLevel::Info,
               "RESPONSE FROM HOST (moveBy): \n"
               "\tResponse status: %d\n"
               "\tDetails: %s",
               status, details.c_str());

    // Listen for a return message
    MessageDTO returnMessage;
    deserializer.deserializeFromStream(returnMessage);
}