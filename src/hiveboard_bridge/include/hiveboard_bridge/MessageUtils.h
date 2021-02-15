#ifndef HIVEBOARD_BRIDGE_MESSAGEUTILS_H
#define HIVEBOARD_BRIDGE_MESSAGEUTILS_H

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
                                     UserCallDestinationDTO moduleDestination,
                                     GenericResponseStatusDTO status,
                                     std::string ackMessage);
} // namespace MessageUtils

#endif // HIVEBOARD_BRIDGE_MESSAGEUTILS_H
