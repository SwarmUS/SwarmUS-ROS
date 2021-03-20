#include "hive_mind_bridge/HiveMindBridgeImpl.h"
#include "mocks/HiveMindHostDeserializerInterfaceMock.h"
#include "mocks/HiveMindHostSerializerInterfaceMock.h"
#include "mocks/MessageHandlerInterfaceMock.h"
#include "mocks/TCPServerInterfaceMock.h"
#include "mocks/ThreadSafeQueueInterfaceMock.h"
#include <gmock/gmock.h>

class HiveMindBridgeImplUnitFixture : public testing::Test {
  protected:
    TCPServerInterfaceMock m_tcpServer;
    HiveMindHostDeserializerInterfaceMock m_deserializer;
    HiveMindHostSerializerInterfaceMock m_serializer;
    HiveMindBridgeImpl* m_hivemindBridge;
    ThreadSafeQueueInterfaceMock<MessageDTO> m_queue;
    MessageHandlerInterfaceMock m_messageHandler;

    void SetUp() {
        m_hivemindBridge = new HiveMindBridgeImpl(m_tcpServer, m_serializer, m_deserializer, m_messageHandler, m_queue);
    }

    void TearDown() { delete m_hivemindBridge; }
};

TEST_F(HiveMindBridgeImplUnitFixture, spinReceiveValidMessage) {
    EXPECT_CALL(m_tcpServer, isClientConnected()).WillOnce(testing::Return(true));
    EXPECT_CALL(m_queue, empty()).WillOnce(testing::Return(false));
    EXPECT_CALL(m_queue, front());
    EXPECT_CALL(m_queue, pop());
    EXPECT_CALL(m_serializer, serializeToStream(testing::_));

    m_hivemindBridge->spin();
}

TEST_F(HiveMindBridgeImplUnitFixture, spinReceiveNoMessage) {
    EXPECT_CALL(m_tcpServer, isClientConnected()).WillOnce(testing::Return(true));
    EXPECT_CALL(m_deserializer, deserializeFromStream(testing::_)).WillOnce(testing::Return(false));

    m_hivemindBridge->spin();
}

//TEST_F(HiveMindBridgeImplUnitFixture, spinClientDisconnected) {
//    EXPECT_CALL(m_tcpServer, isClientConnected()).WillOnce(testing::Return(false));
//    EXPECT_CALL(m_tcpServer, listen());
//
//    m_hivemindBridge->spin();
//}