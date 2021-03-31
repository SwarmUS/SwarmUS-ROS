#include "hive_mind_bridge/MessageHandler.h"
#include <future>

MessageHandler::MessageHandler() {}

MessageHandler::~MessageHandler() {}

MessageDTO MessageHandler::handleFunctionListLengthRequest(uint32_t requestId,
                                                           uint32_t msgDestinationId,
                                                           uint32_t msgSourceId,
                                                           UserCallTargetDTO sourceModule) {

    uint32_t length = m_callbackNames.size();

    return MessageUtils::createFunctionListLengthResponseMessage(requestId, msgDestinationId,
                                                                 msgSourceId, sourceModule, length);
}

MessageDTO MessageHandler::handleFunctionDescriptionRequest(
    uint32_t requestId,
    uint32_t msgDestinationId,
    uint32_t msgSourceId,
    UserCallTargetDTO sourceModule,
    FunctionDescriptionRequestDTO functionDescriptionRequest) {

    uint32_t index = functionDescriptionRequest.getIndex();
    if (index > m_callbackNames.size()) {
        return MessageUtils::createResponseMessage(
            requestId, msgDestinationId, msgSourceId, sourceModule,
            GenericResponseStatusDTO::BadRequest, "Index out of bounds.");
    }

    // m_callbacks and m_callbackNames grow together: the latter is a lookup table for the former.
    // No need to check if the name exists in m_callbacks.
    std::string name = m_callbackNames[index];
    UserCallbackFunctionWrapper cb = m_callbacks[name];

    std::vector<FunctionDescriptionArgumentDTO> args;
    CallbackArgsManifest manifest = cb.getManifest();
    for (auto arg : manifest) {
        FunctionDescriptionArgumentDTO argument(arg.getName().c_str(), arg.getType());
        args.push_back(argument);
    }

    FunctionDescriptionDTO functionDescription(name.c_str(), args.data(), args.size());

    return MessageUtils::createFunctionDescriptionResponseMessage(
        requestId, msgDestinationId, msgSourceId, sourceModule, functionDescription);
}

std::variant<InboundRequestHandle, InboundResponseHandle> MessageHandler::handleMessage(MessageDTO message) {
    uint32_t msgSourceId = message.getSourceId();
    uint32_t msgDestinationId = message.getDestinationId();

    InboundRequestHandle result;

    // Message
    auto request = message.getMessage();

    // Request
    if (std::holds_alternative<RequestDTO>(request)) {
        auto userCallRequest = std::get<RequestDTO>(request).getRequest();
        uint32_t requestId = std::get<RequestDTO>(request).getId();

        // UserCallRequest
        if (std::holds_alternative<UserCallRequestDTO>(userCallRequest)) {
            const auto functionCallRequest =
                std::get<UserCallRequestDTO>(userCallRequest).getRequest();
            UserCallTargetDTO sourceModule =
                std::get<UserCallRequestDTO>(userCallRequest).getSource();

            // FunctionCallRequest
            if (std::holds_alternative<FunctionCallRequestDTO>(functionCallRequest)) {
                FunctionCallRequestDTO function =
                    std::get<FunctionCallRequestDTO>(functionCallRequest);
                std::string functionName = function.getFunctionName();
                CallbackArgs functionArgs = function.getArguments();

                uint16_t argsLength = function.getArgumentsLength();

                auto callback = getCallback(functionName);

                // Call the right callback
                GenericResponseStatusDTO responseStatus = GenericResponseStatusDTO::BadRequest;
                if (callback) {
                    std::shared_future<std::optional<CallbackReturn>> ret =
                        std::async(std::launch::async, callback.value(), functionArgs, argsLength)
                            .share();
                    result.setCallbackReturnContext(ret);
                    result.setCallbackName(functionName);
                    result.setMessageSourceId(msgSourceId);
                    result.setMessageDestinationId(msgDestinationId);
                    result.setSourceModule(sourceModule);

                    responseStatus = GenericResponseStatusDTO::Ok;
                } else {
                    responseStatus = GenericResponseStatusDTO::Unknown;
                    ROS_WARN("Function name \"%s\" was not registered as a callback",
                             functionName.c_str());
                }

                result.setResponse(MessageUtils::createResponseMessage(
                    requestId, msgDestinationId, msgSourceId, sourceModule, responseStatus, ""));

                // FunctionListLengthRequest
            } else if (std::holds_alternative<FunctionListLengthRequestDTO>(functionCallRequest)) {
                result.setResponse(handleFunctionListLengthRequest(requestId, msgDestinationId,
                                                                   msgSourceId, sourceModule));
                // FunctionDescriptionRequest
            } else if (std::holds_alternative<FunctionDescriptionRequestDTO>(functionCallRequest)) {
                result.setResponse(handleFunctionDescriptionRequest(
                    requestId, msgDestinationId, msgSourceId, sourceModule,
                    std::get<FunctionDescriptionRequestDTO>(functionCallRequest)));
            }
        }
    }
    return result;
}

std::optional<uint32_t> MessageHandler::handleGreet(MessageDTO greetMessage) {
    auto greeting = greetMessage.getMessage();

    if (std::holds_alternative<GreetingDTO>(greeting)) {
        return std::get<GreetingDTO>(greeting).getId();
    }

    return {};
}

bool MessageHandler::registerCallback(std::string name, CallbackFunction callback) {
    CallbackArgsManifest manifest;
    return registerCallback(name, callback, manifest);
}

bool MessageHandler::registerCallback(std::string name,
                                      CallbackFunction callback,
                                      CallbackArgsManifest manifest) {
    bool wasOverwritten = false;
    auto existing = m_callbacks.find(name);

    if (existing != m_callbacks.end()) {
        wasOverwritten = true;
    }

    UserCallbackFunctionWrapper cb(callback, manifest);
    m_callbacks[name] = cb;
    m_callbackNames.push_back(name);

    return wasOverwritten;
}

std::optional<CallbackFunction> MessageHandler::getCallback(const std::string& name) {
    auto callback = m_callbacks.find(name);
    if (callback != m_callbacks.end()) {
        return callback->second.getFunction();
    }

    return {};
}