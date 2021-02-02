#include "hiveboard_bridge/MessageHandler.h"
#include "hiveboard_bridge/StreamListener.h"
#include "hiveboard_bridge/TCPServer.h"
#include "hiveboard_bridge/TCPServerMonitor.h"
#include "ros/ros.h"
#include <functional>
#include <hivemind-host/FunctionCallArgumentDTO.h>
#include <hivemind-host/FunctionCallRequestDTO.h>
#include <hivemind-host/FunctionCallResponseDTO.h>
#include <hivemind-host/MessageDTO.h>
#include <hivemind-host/RequestDTO.h>
#include <mutex>
#include <optional>
#include <thread>

#define DEFAULT_TCP_SERVER_PORT 8080

int getTcpServerPort() {
    int port;

    if (!ros::param::get("~TCP_SERVER_PORT", port)) {
        ROS_INFO("No TCP_SERVER_PORT param was given. Using default value %d",
                 DEFAULT_TCP_SERVER_PORT);
        port = DEFAULT_TCP_SERVER_PORT;
    }

    return port;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "hiveboard_bridge");

    //    int port = getTcpServerPort();

    MessageHandler messageHandler;
    //    std::string functionNameStr = "TestFunctionCallRequestDTO";
    //    std::function<void(std::array<FunctionCallArgumentDTO,
    //    FunctionCallRequestDTO::FUNCTION_CALL_ARGUMENTS_MAX_LENGTH>)> testFunction = []() {
    //        ROS_INFO("Test printing from a function pointer");
    //    };
    //    messageHandler.registerCallback(functionNameStr, testFunction);

    FunctionCallRequestDTO functionCallRequestDto("TestFunctionCallRequestDTO", nullptr, 0);
    RequestDTO requestDto(1, functionCallRequestDto);
    MessageDTO m_messageDto(1, 2, requestDto);

    //    const std::variant<std::monostate, RequestDTO>& request = m_messageDto.getMessage();
    //    if (std::holds_alternative<RequestDTO>(request)) {
    //        std::variant<std::monostate, FunctionCallRequestDTO> functionCallRequest =
    //        std::get<RequestDTO>(request).getRequest();
    //
    //        if (std::holds_alternative<FunctionCallRequestDTO>(functionCallRequest)) {
    //    ROS_INFO("Function name (main): %s",
    //    std::get<FunctionCallRequestDTO>(functionCallRequestDto).getFunctionName());
    //        }
    //    }

    messageHandler.handleMessage(m_messageDto);

    //    StreamListener streamListener(port, messageHandler);

    ros::spin();
    return 0;
}