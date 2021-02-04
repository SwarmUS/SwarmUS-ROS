#ifndef HIVEBOARD_BRIDGE_HIVEMINDHOSTDESERIALIZERINTERFACEMOCK_H
#define HIVEBOARD_BRIDGE_HIVEMINDHOSTDESERIALIZERINTERFACEMOCK_H

#include <hivemind-host/IHiveMindHostDeserializer.h>
#include <gmock/gmock.h>

class HiveMindHostDeserializerInterfaceMock : public IHiveMindHostDeserializer {
public:
    ~HiveMindHostDeserializerInterfaceMock() = default;

    MOCK_METHOD((std::variant<std::monostate, MessageDTO>), deserializeFromStream, (), (override));
};

#endif //HIVEBOARD_BRIDGE_HIVEMINDHOSTDESERIALIZERINTERFACEMOCK_H
