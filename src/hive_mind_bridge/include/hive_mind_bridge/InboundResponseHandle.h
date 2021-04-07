#ifndef HIVE_MIND_BRIDGE_INBOUNDRESPONSEHANDLE_H
#define HIVE_MIND_BRIDGE_INBOUNDRESPONSEHANDLE_H

#include <pheromones/GenericResponseDTO.h>
#include <string>

/**
 * Class containing the necessary context to handle an inbound response message
 */
class InboundResponseHandle {
  public:
    InboundResponseHandle();

    InboundResponseHandle(uint32_t responseId,
                          GenericResponseStatusDTO status,
                          std::string details);

    uint32_t getResponseId() const;

    void setResponseId(uint32_t mResponseId);

    GenericResponseStatusDTO getResponseStatus() const;

    void setResponseStatus(GenericResponseStatusDTO mResponseStatus);

    std::string getStatusDetails() const;

    void setStatusDetails(const std::string& mStatusDetails);

  private:
    uint32_t m_responseId; // The ID of the inbound response. Corresponds to the ID of the Request
                           // that was first sent.
    GenericResponseStatusDTO m_responseStatus; // The status of the inbound response
    std::string m_statusDetails; // A verbose explanation of the status
};

#endif // HIVE_MIND_BRIDGE_INBOUNDRESPONSEHANDLE_H
