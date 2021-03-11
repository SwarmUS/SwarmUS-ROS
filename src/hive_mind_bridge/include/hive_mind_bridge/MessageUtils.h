#ifndef HIVEMIND_BRIDGE_MESSAGEUTILS_H
#define HIVEMIND_BRIDGE_MESSAGEUTILS_H

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
     * @return
     */
    MessageDTO createResponseMessage(uint32_t responseId,
                                     uint32_t msgSourceId,
                                     uint32_t msgDestinationId,
                                     UserCallTargetDTO moduleDestination,
                                     GenericResponseStatusDTO status,
                                     std::string ackMessage);

    MessageDTO createFunctionListLengthResponseMessage(uint32_t responseId,
                                                       uint32_t msgSourceId,
                                                       uint32_t msgDestinationId,
                                                       UserCallTargetDTO moduleDestination,
                                                       uint32_t length);
} // namespace MessageUtils

#endif // HIVEMIND_BRIDGE_MESSAGEUTILS_H
