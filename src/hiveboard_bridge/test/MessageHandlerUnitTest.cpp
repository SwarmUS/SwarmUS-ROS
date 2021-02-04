#include "hiveboard_bridge/MessageHandler.h"

#include <functional>
#include <gmock/gmock.h>
#include <hivemind-host/FunctionCallArgumentDTO.h>
#include <hivemind-host/FunctionCallRequestDTO.h>
#include <hivemind-host/MessageDTO.h>
#include <hivemind-host/RequestDTO.h>
#include <optional>
#include <ros/ros.h> // TODO remove this

bool g_testFunctionCalled = false;
int g_testValue1 = 0;
int g_testValue2 = 12;

class MessageHandlerFixture : public testing::Test {
  protected:
    // Declare some test callbacks
    CallbackFunction m_testFunction = [](CallbackArgs args) { g_testFunctionCalled = true; };

    CallbackFunction m_moveByTestCallback = [](CallbackArgs args) {
        g_testValue1 += std::get<int64_t>(args[0].getArgument());
        g_testValue2 -= std::get<int64_t>(args[1].getArgument());
    };

    MessageHandler m_messageHandler;

    // Declare some test messages
    FunctionCallRequestDTO* m_functionCallRequestDto;
    RequestDTO* m_requestDto;

    MessageDTO* m_messageDto;
    FunctionCallRequestDTO* m_nonExistingFunctionCallRequestDto;
    RequestDTO* m_nonExistingRequestDto;
    MessageDTO* m_nonExistingMessageDto;

    FunctionCallArgumentDTO* m_sideEffectArg1;
    FunctionCallArgumentDTO* m_sideEffectArg2;
    FunctionCallRequestDTO* m_sideEffectFunctionCallRequestDto;
    RequestDTO* m_sideEffectRequestDto;

    MessageDTO* m_moveByMessageDto;

    void SetUp() override {
        m_messageHandler.registerCallback("TestFunctionCallRequestDTO", m_testFunction);
        m_messageHandler.registerCallback("MoveBy", m_moveByTestCallback);

        g_testFunctionCalled = false;
        g_testValue1 = 0;
        g_testValue2 = 12;

        // Existing void  function
        m_functionCallRequestDto =
            new FunctionCallRequestDTO("TestFunctionCallRequestDTO", nullptr, 0);
        m_requestDto = new RequestDTO(1, *m_functionCallRequestDto);
        m_messageDto = new MessageDTO(1, 2, *m_requestDto);

        // Nonexisting void function
        m_nonExistingFunctionCallRequestDto = new FunctionCallRequestDTO("NonExisting", nullptr, 0);
        m_nonExistingRequestDto = new RequestDTO(1, *m_nonExistingFunctionCallRequestDto);
        m_nonExistingMessageDto = new MessageDTO(1, 2, *m_nonExistingRequestDto);

        // Testing a callback with side effects
        m_sideEffectArg1 = new FunctionCallArgumentDTO(1);
        m_sideEffectArg2 = new FunctionCallArgumentDTO(5);
        FunctionCallArgumentDTO args[2] = {*m_sideEffectArg1, *m_sideEffectArg2};
        m_sideEffectFunctionCallRequestDto = new FunctionCallRequestDTO("MoveBy", args, 2);
        m_sideEffectRequestDto = new RequestDTO(1, *m_sideEffectFunctionCallRequestDto);
        m_moveByMessageDto = new MessageDTO(1, 2, *m_sideEffectRequestDto);
    }

    void TearDown() override {
        delete m_functionCallRequestDto;
        delete m_requestDto;
        delete m_messageDto;

        delete m_nonExistingFunctionCallRequestDto;
        delete m_nonExistingRequestDto;
        delete m_nonExistingMessageDto;

        delete m_sideEffectArg1;
        delete m_sideEffectArg2;
        delete m_sideEffectFunctionCallRequestDto;
        delete m_sideEffectRequestDto;
        delete m_moveByMessageDto;
    }
};

TEST_F(MessageHandlerFixture, testGetCallbackSuccess) {
    ASSERT_TRUE(m_messageHandler.getCallback("TestFunctionCallRequestDTO"));
}

TEST_F(MessageHandlerFixture, testGetCallbackFail) {
    ASSERT_FALSE(m_messageHandler.getCallback("Nonexisting"));
}

TEST_F(MessageHandlerFixture, testHandleMessageVoidFunctionSuccess) {
    ASSERT_TRUE(m_messageHandler.handleMessage(*m_messageDto));
    ASSERT_TRUE(g_testFunctionCalled);
}

TEST_F(MessageHandlerFixture, TestHandleMessageMoveByFunctionSuccess) {
    ASSERT_TRUE(m_messageHandler.handleMessage(*m_moveByMessageDto));
    ASSERT_EQ(g_testValue1, 1);
    ASSERT_EQ(g_testValue2, 7);
}

TEST_F(MessageHandlerFixture, testHandleMessageFail) {
    ASSERT_FALSE(m_messageHandler.handleMessage(*m_nonExistingMessageDto));
}