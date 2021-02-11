#ifndef HIVEBOARD_BRIDGE_MESSAGEUTILS_H
#define HIVEBOARD_BRIDGE_MESSAGEUTILS_H

#include <hivemind-host/FunctionCallResponseDTO.h>
#include <hivemind-host/MessageDTO.h>
#include <hivemind-host/ResponseDTO.h>
#include <hivemind-host/UserCallResponseDTO.h>

namespace MessageUtils {
    MessageDTO createResponseMessage(uint32_t responseId,
                                     uint32_t msgSourceId,
                                     uint32_t msgDestinationId,
                                     UserCallDestinationDTO moduleDestination,
                                     GenericResponseStatusDTO status,
                                     std::string ackMessage);
}

#endif // HIVEBOARD_BRIDGE_MESSAGEUTILS_H
