#ifndef HIVE_MIND_BRIDGE_INBOUNDRESPONSEHANDLE_H
#define HIVE_MIND_BRIDGE_INBOUNDRESPONSEHANDLE_H

#include  <hivemind-host/GenericResponseDTO.h>

class InboundResponseHandle {
public:
    InboundResponseHandle(uint32_t responseId, GenericResponseStatusDTO status, std::string details);

    uint32_t getResponseId() const;

    void setResponseId(uint32_t mResponseId);

    GenericResponseStatusDTO getResponseStatus() const;

    void setResponseStatus(GenericResponseStatusDTO mResponseStatus);

    std::string getStatusDetails() const;

    void setStatusDetails(const std::string &mStatusDetails);

private:
    uint32_t m_responseId;
    GenericResponseStatusDTO m_responseStatus;
    std::string m_statusDetails;
};

#endif //HIVE_MIND_BRIDGE_INBOUNDRESPONSEHANDLE_H
