#include "hive_mind_bridge/MessageUtils.h"

MessageDTO MessageUtils::createResponseMessage(uint32_t responseId,
                                               uint32_t msgSourceId,
                                               uint32_t msgDestinationId,
                                               UserCallTargetDTO moduleDestination,
                                               GenericResponseStatusDTO status,
                                               std::string ackMessage) {
    FunctionCallResponseDTO functionCallResponse(status, ackMessage.c_str());
    UserCallResponseDTO userCallResponse(UserCallTargetDTO::HOST, moduleDestination,
                                         functionCallResponse);
    ResponseDTO response(responseId, userCallResponse);
    MessageDTO responseMessage(msgSourceId, msgDestinationId, response);

    return responseMessage;
}

MessageDTO MessageUtils::createFunctionListLengthResponseMessage(
    uint32_t responseId,
    uint32_t msgSourceId,
    uint32_t msgDestinationId,
    UserCallTargetDTO moduleDestination,
    uint32_t length) {

    FunctionListLengthResponseDTO functionListLengthResponse(length);
    UserCallResponseDTO userCallResponse(UserCallTargetDTO::HOST, moduleDestination,
                                         functionListLengthResponse);
    ResponseDTO response(responseId, userCallResponse);
    MessageDTO responseMessage(msgSourceId, msgDestinationId, response);

    return responseMessage;
}

MessageDTO MessageUtils::createFunctionDescriptionResponseMessage(
    uint32_t responseId,
    uint32_t msgSourceId,
    uint32_t msgDestinationId,
    UserCallTargetDTO moduleDestination,
    FunctionDescriptionDTO functionDescription) {

    FunctionDescriptionResponseDTO functionDescriptionResponse(functionDescription);
    UserCallResponseDTO userCallResponse(UserCallTargetDTO::HOST, moduleDestination,
                                         functionDescriptionResponse);
    ResponseDTO response(responseId, userCallResponse);
    MessageDTO responseMessage(msgSourceId, msgDestinationId, response);

    return responseMessage;
}

uint8_t MessageUtils::createFunctionCallArguments(CallbackArgs args, FunctionCallArgumentDTO* functionCallArgument) {
    functionCallArgument = args.data();
    return args.size();
}

MessageDTO MessageUtils::createFunctionCallRequest(uint32_t msgSourceId,
                                     uint32_t msgDestinationId,
                                     uint32_t requestId,
                                     UserCallTargetDTO moduleDestination,
                                     std::string callbackName,
                                     CallbackArgs args) {
    FunctionCallArgumentDTO *functionCallArgument;
    uint8_t size = createFunctionCallArguments(args, functionCallArgument);

    FunctionCallRequestDTO functionCallRequest(callbackName.c_str(), functionCallArgument, size);

    UserCallRequestDTO userCallRequest(UserCallTargetDTO::HOST, moduleDestination,
                                       functionCallRequest);
    RequestDTO RequestDTO(1, userCallRequest);
    MessageDTO message(1, 2, RequestDTO);

    return message;
}