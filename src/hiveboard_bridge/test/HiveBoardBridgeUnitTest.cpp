#include <gmock/gmock.h>
#include "mocks/HiveMindHostDeserializerInterfaceMock.h"
#include "mocks/HiveMindHostSerializerInterfaceMock.h"
#include "mocks/TCPServerInterfaceMock.h"
#include "hiveboard_bridge/HiveBoardBridge.h"

class HiveBoardBridgeUnitFixture : public testing::Test {
protected:
    TCPServerInterfaceMock m_tcpServer;
    HiveMindHostDeserializerInterfaceMock m_deserializer;
    HiveMindHostSerializerInterfaceMock m_serializer;
    HiveBoardBridge* m_hiveboardBridge;

    void SetUp() {
        m_hiveboardBridge = new HiveBoardBridge(m_tcpServer, m_serializer, m_deserializer);
    }

    void TearDown() {
        delete m_hiveboardBridge;
    }
};

TEST_F(HiveBoardBridgeUnitFixture, spinReceiveValidMessage) {
    EXPECT_CALL(m_tcpServer, isClientConnected()).WillOnce(testing::Return(true));
    EXPECT_CALL(m_deserializer, deserializeFromStream(testing::_)).WillOnce(testing::Return(true));

    m_hiveboardBridge->spin();
}

TEST_F(HiveBoardBridgeUnitFixture, spinReceiveNoMessage) {
    EXPECT_CALL(m_tcpServer, isClientConnected()).WillOnce(testing::Return(true));
    EXPECT_CALL(m_deserializer, deserializeFromStream(testing::_)).WillOnce(testing::Return(false));

    m_hiveboardBridge->spin();
}