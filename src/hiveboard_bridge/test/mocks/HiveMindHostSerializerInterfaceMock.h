#ifndef HIVEBOARD_BRIDGE_HIVEMINDHOSTSERIALIZERINTERFACEMOCK_H
#define HIVEBOARD_BRIDGE_HIVEMINDHOSTSERIALIZERINTERFACEMOCK_H

#include <gmock/gmock.h>
#include <hivemind-host/IHiveMindHostSerializer.h>

class HiveMindHostSerializerInterfaceMock : public IHiveMindHostSerializer {
  public:
    ~HiveMindHostSerializerInterfaceMock() = default;

    MOCK_METHOD((bool), serializeToStream, (const MessageDTO& message), (override));
};

#endif // HIVEBOARD_BRIDGE_HIVEMINDHOSTSERIALIZERINTERFACEMOCK_H
