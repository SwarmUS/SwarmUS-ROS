#include "hiveboard_bridge/MessageHandler.h"

MessageHandler::MessageHandler() {}

MessageHandler::~MessageHandler() {}

MessageDTO MessageHandler::handleMessage(MessageDTO message) {
    GenericResponseStatusDTO responseStatus = GenericResponseStatusDTO::BadRequest;
    uint32_t msgSourceId = message.getSourceId();
    uint32_t msgDestinationId = message.getDestinationId();
    uint32_t requestId = 0;

    // Message
    auto request = message.getMessage();

    // Request
    if (std::holds_alternative<RequestDTO>(request)) {
        std::variant<std::monostate, UserCallRequestDTO> userCallRequest =
            std::get<RequestDTO>(request).getRequest();
        requestId = std::get<RequestDTO>(request).getId();

        // UserCallRequest
        if (std::holds_alternative<UserCallRequestDTO>(userCallRequest)) {
            std::variant<std::monostate, FunctionCallRequestDTO> functionCallRequest =
                std::get<UserCallRequestDTO>(userCallRequest).getRequest();

            // FunctionCallRequest
            if (std::holds_alternative<FunctionCallRequestDTO>(functionCallRequest)) {
                FunctionCallRequestDTO function =
                    std::get<FunctionCallRequestDTO>(functionCallRequest);
                std::string functionName = function.getFunctionName();
                CallbackArgs functionArgs = function.getArguments();

                uint16_t argsLength = function.getArgumentsLength();

                auto callback = getCallback(functionName);

                // Call the right callback
                if (callback) {
                    CallbackContext ctx;
                    ctx.compoundSourceId = message.getSourceId();
                    ctx.compoundDestinationId = message.getDestinationId();
                    ctx.moduleDestinationId =
                        std::get_if<UserCallRequestDTO>(&userCallRequest)->getDestination();
                    ctx.expectedResponseId = std::get_if<RequestDTO>(&request)->getId();

                    callback.value()(functionArgs, argsLength, ctx);
                    responseStatus = GenericResponseStatusDTO::Ok;
                } else {
                    responseStatus = GenericResponseStatusDTO::Unknown;
                    ROS_WARN("Function name \"%s\" was not registered as a callback",
                             functionName.c_str());
                }
            }
        }
    }

    if (responseStatus != GenericResponseStatusDTO::Ok) {
        ROS_WARN("Message handling failed");
    }

    return MessageUtils::createResponseMessage(requestId, msgSourceId, msgDestinationId,
                                               UserCallDestinationDTO::BUZZ, responseStatus, "");
}

bool MessageHandler::registerCallback(std::string name, CallbackFunction callback) {
    bool wasOverwritten = m_callbacks[name] != nullptr;
    m_callbacks[name] = callback;
    return wasOverwritten;
}

std::optional<CallbackFunction> MessageHandler::getCallback(const std::string& name) {
    auto callback = m_callbacks.find(name);
    if (callback != m_callbacks.end()) {
        return callback->second;
    }

    return {};
}