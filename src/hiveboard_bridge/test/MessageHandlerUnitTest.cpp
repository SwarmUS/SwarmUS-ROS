#include "hiveboard_bridge/MessageHandler.h"

#include <functional>
#include <gmock/gmock.h>
#include <hivemind-host/FunctionCallArgumentDTO.h>
#include <hivemind-host/FunctionCallRequestDTO.h>
#include <hivemind-host/MessageDTO.h>
#include <hivemind-host/RequestDTO.h>
#include <optional>
#include <ros/ros.h> // TODO remove this

bool m_testFunctionCalled;
int m_testValue1;
int m_testValue2;

class MessageHandlerFixture : public testing::Test {
protected:
    // Declare some test callbacks
    std::function<void(callbackArgs)> m_testFunction = [](callbackArgs args) { m_testFunctionCalled = true; };

    std::function<void(callbackArgs)> m_moveByTestCallback = [](callbackArgs args) {
        ROS_WARN("MOVE BY (%d, %d)", args[0], args[1]);
        m_testValue1 += args[0];
        m_testValue2 -= args[1];
    };

    MessageHandler m_messageHandler;
    // Declare some test messages
    FunctionCallRequestDTO* m_functionCallRequestDto;
    RequestDTO* m_requestDto;

    MessageDTO* m_messageDto;
    FunctionCallRequestDTO* m_nonExistingFunctionCallRequestDto;
    RequestDTO* m_nonExistingRequestDto;

    MessageDTO* m_nonExistingMessageDto;
    FunctionCallArgumentDTO* m_moveByArgumentX;
    FunctionCallArgumentDTO* m_moveByArgumentY;
    FunctionCallRequestDTO* m_moveByFunctionCallRequestDto;
    RequestDTO* m_moveByRequestDto;

    MessageDTO* m_moveByMessageDto;

    void SetUp() override {
        m_messageHandler.registerCallback("TestFunctionCallRequestDTO", m_testFunction);
        m_messageHandler.registerCallback("MoveBy", m_moveByTestCallback);
//        m_sideEffectTester.reset();
        m_testFunctionCalled = false;
        m_testValue1 = 0;
        m_testValue2 = 12;

        // Existing void  function
        m_functionCallRequestDto =
            new FunctionCallRequestDTO("TestFunctionCallRequestDTO", nullptr, 0);
        m_requestDto = new RequestDTO(1, *m_functionCallRequestDto);
        m_messageDto = new MessageDTO(1, 2, *m_requestDto);

        // Nonexisting void function
        m_nonExistingFunctionCallRequestDto = new FunctionCallRequestDTO("NonExisting", nullptr, 0);
        m_nonExistingRequestDto = new RequestDTO(1, *m_nonExistingFunctionCallRequestDto);
        m_nonExistingMessageDto = new MessageDTO(1, 2, *m_nonExistingRequestDto);

        // Testing a robotMoveBy(x, y) function
        m_moveByArgumentX = new FunctionCallArgumentDTO(1);
        m_moveByArgumentY = new FunctionCallArgumentDTO(5);
        FunctionCallArgumentDTO args[2] = {*m_moveByArgumentX, *m_moveByArgumentY};
        m_moveByFunctionCallRequestDto = new FunctionCallRequestDTO("MoveBy", args, 2);
        m_moveByRequestDto = new RequestDTO(1, *m_moveByFunctionCallRequestDto);
        m_moveByMessageDto = new MessageDTO(1, 2, *m_moveByRequestDto);
    }

    void TearDown() override {
        delete m_functionCallRequestDto;
        delete m_requestDto;
        delete m_messageDto;

        delete m_nonExistingFunctionCallRequestDto;
        delete m_nonExistingRequestDto;
        delete m_nonExistingMessageDto;

        delete m_moveByArgumentX;
        delete m_moveByArgumentY;
        delete m_moveByFunctionCallRequestDto;
        delete m_moveByRequestDto;
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
    //    EXPECT_CALL(MessageHandlerFixture, m_testFunction());

    ASSERT_TRUE(m_messageHandler.handleMessage(*m_messageDto));
    ASSERT_TRUE(m_testFunctionCalled);
}

TEST_F(MessageHandlerFixture, TestHandleMessageMoveByFunctionSuccess) {
    ASSERT_TRUE(m_messageHandler.handleMessage(*m_moveByMessageDto));
    ASSERT_EQ(m_testValue1, 1);
    ASSERT_EQ(m_testValue2, 7);
}

TEST_F(MessageHandlerFixture, testHandleMessageFail) {
    ASSERT_FALSE(m_messageHandler.handleMessage(*m_nonExistingMessageDto));
}