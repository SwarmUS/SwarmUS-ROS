#include "hive_mind_bridge/HiveMindBridgeImpl.h"
#include "mocks/HiveMindHostDeserializerInterfaceMock.h"
#include "mocks/HiveMindHostSerializerInterfaceMock.h"
#include "mocks/MessageHandlerInterfaceMock.h"
#include "mocks/TCPServerInterfaceMock.h"
#include "mocks/ThreadSafeQueueInterfaceMock.h"
#include <gmock/gmock.h>

std::function<std::optional<CallbackArgs>()> validCallbackWithInstantReturn =
    []() -> std::optional<CallbackArgs> {
    CallbackArgs returnArgs;
    returnArgs[0] = FunctionCallArgumentDTO(1.0f);

    return returnArgs;
};

std::function<std::optional<CallbackArgs>()> validCallbackWithoutInstantReturn =
    []() -> std::optional<CallbackArgs> { return {}; };

class HiveMindBridgeImplUnitFixture : public testing::Test {
  protected:
    TCPServerInterfaceMock m_tcpServer;
    HiveMindHostDeserializerInterfaceMock m_deserializer;
    HiveMindHostSerializerInterfaceMock m_serializer;
    HiveMindBridgeImpl* m_hivemindBridge;
    ThreadSafeQueueInterfaceMock<MessageDTO> m_queue;
    MessageHandlerInterfaceMock m_messageHandler;

    MessageHandlerResult validResultWithReturn;
    MessageDTO dummyResponseMessage = MessageUtils::createResponseMessage(
        1, 1, 1, UserCallTargetDTO::UNKNOWN, GenericResponseStatusDTO::Ok, "");

    void SetUp() {
        m_hivemindBridge = new HiveMindBridgeImpl(m_tcpServer, m_serializer, m_deserializer,
                                                  m_messageHandler, m_queue);
    }

    void TearDown() { delete m_hivemindBridge; }
};

TEST_F(HiveMindBridgeImplUnitFixture, spinInstantaneousCallback_WithReturn) {
    // Given
    validResultWithReturn.setFuture(
        std::async(std::launch::async, validCallbackWithInstantReturn).share());
    validResultWithReturn.setResponse(dummyResponseMessage);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(250)); // Just to make sure the callback ends before test

    // When
    EXPECT_CALL(m_tcpServer, isClientConnected()).WillOnce(testing::Return(true));
    EXPECT_CALL(m_queue, empty()).WillOnce(testing::Return(false));
    EXPECT_CALL(m_queue, front());
    EXPECT_CALL(m_messageHandler, handleMessage(testing::_))
        .WillOnce(testing::Return(validResultWithReturn));
    EXPECT_CALL(m_queue, pop());
    EXPECT_CALL(m_serializer, serializeToStream(testing::_)).Times(2); // ack + return

    m_hivemindBridge->spin();

    // Then
}

TEST_F(HiveMindBridgeImplUnitFixture, spinInstantaneousCallback_WithoutReturn) {
    // Given
    validResultWithReturn.setFuture(
        std::async(std::launch::async, validCallbackWithoutInstantReturn).share());
    validResultWithReturn.setResponse(dummyResponseMessage);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(250)); // Just to make sure the callback ends before test

    // When
    EXPECT_CALL(m_tcpServer, isClientConnected()).WillOnce(testing::Return(true));
    EXPECT_CALL(m_queue, empty()).WillOnce(testing::Return(false));
    EXPECT_CALL(m_queue, front());
    EXPECT_CALL(m_messageHandler, handleMessage(testing::_))
        .WillOnce(testing::Return(validResultWithReturn));
    EXPECT_CALL(m_queue, pop());
    EXPECT_CALL(m_serializer, serializeToStream(testing::_)).Times(1); // ack only

    m_hivemindBridge->spin();

    // Then
}
