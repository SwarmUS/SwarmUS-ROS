#ifndef HIVEMIND_BRIDGE_HIVEMINDHOSTDESERIALIZERINTERFACEMOCK_H
#define HIVEMIND_BRIDGE_HIVEMINDHOSTDESERIALIZERINTERFACEMOCK_H

#include <gmock/gmock.h>
#include <pheromones/IHiveMindHostDeserializer.h>

class HiveMindHostDeserializerInterfaceMock : public IHiveMindHostDeserializer {
  public:
    ~HiveMindHostDeserializerInterfaceMock() = default;

    MOCK_METHOD(bool, deserializeFromStream, (MessageDTO & message), (override));
};

#endif // HIVEMIND_BRIDGE_HIVEMINDHOSTDESERIALIZERINTERFACEMOCK_H
