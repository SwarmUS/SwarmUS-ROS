#include "hiveboard_bridge/HiveMindBridgeImpl.h"
#include "mocks/HiveMindHostDeserializerInterfaceMock.h"
#include "mocks/HiveMindHostSerializerInterfaceMock.h"
#include "mocks/TCPServerInterfaceMock.h"
#include <gmock/gmock.h>

class HiveBoardBridgeImplUnitFixture : public testing::Test {
  protected:
    TCPServerInterfaceMock m_tcpServer;
    HiveMindHostDeserializerInterfaceMock m_deserializer;
    HiveMindHostSerializerInterfaceMock m_serializer;
    HiveBoardBridgeImpl* m_hiveboardBridge;

    void SetUp() {
        m_hiveboardBridge = new HiveBoardBridgeImpl(m_tcpServer, m_serializer, m_deserializer);
    }

    void TearDown() { delete m_hiveboardBridge; }
};

TEST_F(HiveBoardBridgeImplUnitFixture, spinReceiveValidMessage) {
    EXPECT_CALL(m_tcpServer, isClientConnected()).WillOnce(testing::Return(true));
    EXPECT_CALL(m_deserializer, deserializeFromStream(testing::_)).WillOnce(testing::Return(true));
    EXPECT_CALL(m_serializer, serializeToStream(testing::_));

    m_hiveboardBridge->spin();
}

TEST_F(HiveBoardBridgeImplUnitFixture, spinReceiveNoMessage) {
    EXPECT_CALL(m_tcpServer, isClientConnected()).WillOnce(testing::Return(true));
    EXPECT_CALL(m_deserializer, deserializeFromStream(testing::_)).WillOnce(testing::Return(false));

    m_hiveboardBridge->spin();
}

TEST_F(HiveBoardBridgeImplUnitFixture, spinClientDisconnected) {
    EXPECT_CALL(m_tcpServer, isClientConnected()).WillOnce(testing::Return(false));
    EXPECT_CALL(m_tcpServer, listen());

    m_hiveboardBridge->spin();
}