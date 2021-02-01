#include "hiveboard_bridge/MessageHandler.h"
//#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <functional>
#include <optional>

class MessageHandlerFixture : public testing::Test {
protected:
    MessageHandler m_messageHandler;
    std::function<void()> m_testFunction = []() {
        int a = 0;
    };

    void SetUp() override {
        m_messageHandler.registerCallback("Test", m_testFunction);
    }

    void TearDown() override {}
};

TEST(MessageHandler, dummy) {
    ASSERT_TRUE(true);
}

TEST_F(MessageHandlerFixture, testGetCallbackSuccess) {
    ASSERT_TRUE(m_messageHandler.getCallback("Test"));
}

TEST_F(MessageHandlerFixture, testGetCallbackFail) {
    ASSERT_FALSE(m_messageHandler.getCallback("Nonexisting"));
}

TEST_F(MessageHandlerFixture, testHandleMessageSuccess) {
//    EXPECT_CALL(MessageHandlerFixture, m_testFunction());

    ASSERT_TRUE(m_messageHandler.handleMessage("Test"));

}

TEST_F(MessageHandlerFixture, testHandleMessageFail) {
    ASSERT_FALSE(m_messageHandler.handleMessage("Nonexisting"));
}