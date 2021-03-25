#ifndef HIVEMIND_BRIDGE_MESSAGEUTILS_H
#define HIVEMIND_BRIDGE_MESSAGEUTILS_H

#include "hive_mind_bridge/UserCallbackFunctionWrapper.h"
#include <hivemind-host/FunctionCallResponseDTO.h>
#include <hivemind-host/MessageDTO.h>
#include <hivemind-host/ResponseDTO.h>
#include <hivemind-host/UserCallResponseDTO.h>

/**
 * Utilitary functions for message creation
 */
namespace MessageUtils {
    /**
     * Create a response message containing some parameters
     * @param responseId Id of the request responded to
     * @param msgSourceId Source of the response. Should correspond to the ID of this device.
     * @param msgDestinationId Destination of the response.
     * @param moduleDestination The module to which the response should be addressed.
     * @param status The status of the processing of the message
     * @param ackMessage A string containing some details.
     * @return the created message
     */
    MessageDTO createResponseMessage(uint32_t responseId,
                                     uint32_t msgSourceId,
                                     uint32_t msgDestinationId,
                                     UserCallTargetDTO moduleDestination,
                                     GenericResponseStatusDTO status,
                                     std::string ackMessage);

    /**
     * Create a response message for a FunctionListLength request
     * @param responseId Id of the request responded to
     * @param msgSourceId Source of the response. Should correspond to the ID of this device.
     * @param msgDestinationId Destination of the response.
     * @param moduleDestination The module to which the response should be addressed.
     * @param length the length of the function list
     * @return the created message
     */
    MessageDTO createFunctionListLengthResponseMessage(uint32_t responseId,
                                                       uint32_t msgSourceId,
                                                       uint32_t msgDestinationId,
                                                       UserCallTargetDTO moduleDestination,
                                                       uint32_t length);

    /**
     * Create a response message for a FunctionDescription request
     * @param responseId Id of the request responded to
     * @param msgSourceId Source of the response. Should correspond to the ID of this device.
     * @param msgDestinationId Destination of the response.
     * @param moduleDestination The module to which the response should be addressed.
     * @param functionDescription The description of the function
     * @return the created message
     */
    MessageDTO createFunctionDescriptionResponseMessage(uint32_t responseId,
                                                        uint32_t msgSourceId,
                                                        uint32_t msgDestinationId,
                                                        UserCallTargetDTO moduleDestination,
                                                        FunctionDescriptionDTO functionDescription);

    /**
     * Create a message with a Function call request with arguments
     * @param msgSourceId The ID of this device
     * @param msgDestinationId The ID of the target device
     * @param requestId The ID of the request
     * @param moduleDestination The module to send the message to
     * @param callbackName The name of the function
     * @param args The args to pass to the function
     * @return the created message
     */
    MessageDTO createFunctionCallRequest(uint32_t msgSourceId,
                                         uint32_t msgDestinationId,
                                         uint32_t requestId,
                                         UserCallTargetDTO moduleDestination,
                                         std::string callbackName,
                                         CallbackArgs args);

    /**
     * Create a message with a Function call request without arguments
     * @param msgSourceId The ID of this device
     * @param msgDestinationId The ID of the target device
     * @param requestId The ID of the request
     * @param moduleDestination The module to send the message to
     * @param callbackName The name of the function
     * @return the created message
     */
    MessageDTO createFunctionCallRequest(uint32_t msgSourceId,
                                         uint32_t msgDestinationId,
                                         uint32_t requestId,
                                         UserCallTargetDTO moduleDestination,
                                         std::string callbackName);

    /**
     * Returns a suitable value to use as a requestId;
     */
    uint32_t generateRandomId();

} // namespace MessageUtils

#endif // HIVEMIND_BRIDGE_MESSAGEUTILS_H
