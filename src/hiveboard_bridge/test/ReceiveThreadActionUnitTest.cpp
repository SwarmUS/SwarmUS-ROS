#include "hiveboard_bridge/ReceiveAction.h"
#include "mocks/HiveMindHostDeserializerInterfaceMock.h"
#include "mocks/MessageHandlerInterfaceMock.h"
#include <gmock/gmock.h>

class ReceiveActionUnitFixture : public testing::Test {
  protected:
    HiveMindHostDeserializerInterfaceMock m_deserializer;
    MessageHandlerInterfaceMock m_messageHandler;
    ReceiveAction* m_receiveAction;

    FunctionCallRequestDTO* m_functionCallRequestDto;
    RequestDTO* m_requestDto;
    MessageDTO* m_messageDto;
    std::variant<std::monostate, MessageDTO>* m_messageVariant;
    std::variant<std::monostate, MessageDTO> m_messageVariantEmpty;

    void SetUp() {
        m_functionCallRequestDto =
            new FunctionCallRequestDTO("TestFunctionCallRequestDTO", nullptr, 0);
        m_requestDto = new RequestDTO(1, *m_functionCallRequestDto);
        m_messageDto = new MessageDTO(1, 2, *m_requestDto);
        m_messageVariant = new std::variant<std::monostate, MessageDTO>(*m_messageDto);

        m_receiveAction = new ReceiveAction(m_deserializer, m_messageHandler);
    }

    void TearDown() {
        delete m_functionCallRequestDto;
        delete m_requestDto;
        delete m_messageDto;
        delete m_messageVariant;

        delete m_receiveAction;
    }
};

TEST_F(ReceiveActionUnitFixture, receiveValidMessage) {
    EXPECT_CALL(m_deserializer, deserializeFromStream())
        .WillOnce(testing::Return(*m_messageVariant));
    EXPECT_CALL(m_messageHandler, handleMessage(testing::_)).WillOnce(testing::Return(true));

    m_receiveAction->doAction();
}

TEST_F(ReceiveActionUnitFixture, receiveNoMessage) {
    EXPECT_CALL(m_deserializer, deserializeFromStream())
        .WillOnce(testing::Return(m_messageVariantEmpty));
    EXPECT_CALL(m_messageHandler, handleMessage(testing::_)).Times(0);

    m_receiveAction->doAction();
}