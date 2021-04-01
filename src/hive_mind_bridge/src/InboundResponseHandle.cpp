#include <string>
#include "hive_mind_bridge/InboundResponseHandle.h"

InboundResponseHandle::InboundResponseHandle() {}

InboundResponseHandle::InboundResponseHandle(uint32_t responseId, GenericResponseStatusDTO status, std::string details) :
    m_responseId(responseId),
    m_responseStatus(status),
    m_statusDetails(details) {}

uint32_t InboundResponseHandle::getResponseId() const {
    return m_responseId;
}

void InboundResponseHandle::setResponseId(uint32_t mResponseId) {
    m_responseId = mResponseId;
}

GenericResponseStatusDTO InboundResponseHandle::getResponseStatus() const {
    return m_responseStatus;
}

void InboundResponseHandle::setResponseStatus(GenericResponseStatusDTO mResponseStatus) {
    m_responseStatus = mResponseStatus;
}

std::string InboundResponseHandle::getStatusDetails() const {
    return m_statusDetails;
}

void InboundResponseHandle::setStatusDetails(const std::string &mStatusDetails) {
    m_statusDetails = mStatusDetails;
}
