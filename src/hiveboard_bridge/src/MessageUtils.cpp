#include "hiveboard_bridge/MessageUtils.h"

MessageDTO MessageUtils::createResponseMessage(uint32_t responseId,
                                               uint32_t msgSourceId,
                                               uint32_t msgDestinationId,
                                               UserCallTargetDTO moduleDestination,
                                               GenericResponseStatusDTO status,
                                               std::string ackMessage) {
    FunctionCallResponseDTO functionCallResponse(status, ackMessage.c_str());
    UserCallResponseDTO userCallResponse(UserCallTargetDTO::HOST, moduleDestination, functionCallResponse);
    ResponseDTO response(responseId, userCallResponse);
    MessageDTO responseMessage(msgSourceId, msgDestinationId, response);

    return responseMessage;
}