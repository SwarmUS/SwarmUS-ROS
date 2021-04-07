#ifndef HIVEMIND_BRIDGE_HIVEMINDHOSTSERIALIZERINTERFACEMOCK_H
#define HIVEMIND_BRIDGE_HIVEMINDHOSTSERIALIZERINTERFACEMOCK_H

#include <gmock/gmock.h>
#include <pheromones/IHiveMindHostSerializer.h>

class HiveMindHostSerializerInterfaceMock : public IHiveMindHostSerializer {
  public:
    ~HiveMindHostSerializerInterfaceMock() = default;

    MOCK_METHOD((bool), serializeToStream, (const MessageDTO& message), (override));
};

#endif // HIVEMIND_BRIDGE_HIVEMINDHOSTSERIALIZERINTERFACEMOCK_H
