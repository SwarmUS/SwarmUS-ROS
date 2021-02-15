#ifndef HIVEBOARD_BRIDGE_HIVEMINDHOSTDESERIALIZERINTERFACEMOCK_H
#define HIVEBOARD_BRIDGE_HIVEMINDHOSTDESERIALIZERINTERFACEMOCK_H

#include <gmock/gmock.h>
#include <hivemind-host/IHiveMindHostDeserializer.h>

class HiveMindHostDeserializerInterfaceMock : public IHiveMindHostDeserializer {
  public:
    ~HiveMindHostDeserializerInterfaceMock() = default;

    MOCK_METHOD(bool, deserializeFromStream, (MessageDTO & message), (override));
};

#endif // HIVEBOARD_BRIDGE_HIVEMINDHOSTDESERIALIZERINTERFACEMOCK_H
