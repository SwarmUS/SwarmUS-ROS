#include "hive_mind_bridge/MessageHandler.h"

MessageHandler::MessageHandler() {}

MessageHandler::~MessageHandler() {}

MessageDTO MessageHandler::handleFunctionListLengthRequest(uint32_t requestId,
                                                           uint32_t msgDestinationId,
                                                           uint32_t msgSourceId,
                                                           UserCallTargetDTO sourceModule) {

    uint32_t length = m_callbackNames.size();

    return MessageUtils::createFunctionListLengthResponseMessage(requestId, msgDestinationId, msgSourceId, sourceModule, length);
}

MessageDTO MessageHandler::handleFunctionDescriptionRequest(uint32_t requestId,
                                                            uint32_t msgDestinationId,
                                                            uint32_t msgSourceId,
                                                            UserCallTargetDTO sourceModule) {

}

MessageDTO MessageHandler::handleMessage(MessageDTO message) {
    GenericResponseStatusDTO responseStatus = GenericResponseStatusDTO::BadRequest;
    uint32_t msgSourceId = message.getSourceId();
    uint32_t msgDestinationId = message.getDestinationId();
    uint32_t requestId = 0;
    UserCallTargetDTO sourceModule = UserCallTargetDTO::UNKNOWN;

    // Message
    auto request = message.getMessage();

    // Request
    if (std::holds_alternative<RequestDTO>(request)) {
        auto userCallRequest = std::get<RequestDTO>(request).getRequest();
        requestId = std::get<RequestDTO>(request).getId();

        // UserCallRequest
        if (std::holds_alternative<UserCallRequestDTO>(userCallRequest)) {
            const auto functionCallRequest = std::get<UserCallRequestDTO>(userCallRequest).getRequest();
            sourceModule = std::get<UserCallRequestDTO>(userCallRequest).getSource();

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
                    callback.value()(functionArgs, argsLength);
                    responseStatus = GenericResponseStatusDTO::Ok;
                } else {
                    responseStatus = GenericResponseStatusDTO::Unknown;
                    ROS_WARN("Function name \"%s\" was not registered as a callback",
                             functionName.c_str());
                }
            // FunctionListLengthRequest
            } else if (std::holds_alternative<FunctionListLengthRequestDTO>(functionCallRequest)) {
                // TODO change the return statement for something more std
                return handleFunctionListLengthRequest(requestId, msgDestinationId, msgSourceId, sourceModule);
            // FunctionDescriptionRequest
            } else if (std::holds_alternative<FunctionDescriptionRequestDTO>(functionCallRequest)) {
                handleFunctionDescriptionRequest(requestId, msgDestinationId, msgSourceId, sourceModule);
            }
        }
    }

    if (responseStatus != GenericResponseStatusDTO::Ok) {
        ROS_WARN("Message handling failed");
    }

    return MessageUtils::createResponseMessage(requestId, msgDestinationId, msgSourceId,
                                               sourceModule, responseStatus, "");
}

bool MessageHandler::registerCallback(std::string name, CallbackFunction callback) {
    bool wasOverwritten = m_callbacks[name] != nullptr;
    m_callbacks[name] = callback;
    m_callbackNames.push_back(name);

    return wasOverwritten;
}

std::optional<CallbackFunction> MessageHandler::getCallback(const std::string& name) {
    auto callback = m_callbacks.find(name);
    if (callback != m_callbacks.end()) {
        return callback->second;
    }

    return {};
}