#include "hiveboard_bridge/StreamListener.h"
#include "mocks/HiveMindHostDeserializerInterfaceMock.h"
#include "mocks/MessageHandlerInterfaceMock.h"
#include <gmock/gmock.h>

class StreamListenerUUT : public StreamListener {

};

class StreamListenerUnitFixture : public testing::Test {
protected:
    HiveMindHostDeserializerInterfaceMock m_deserializer;
    MessageHandlerInterfaceMock m_messageHandler;
    StreamListenerUUT* m_streamListener;

    void SetUp() {
        m_streamListener = new StreamListenerUUT(m_deserializer, m_messageHandler);
    }

    void TearDown() {

    }
};

TEST_F(StreamListenerUnitFixture, receiveValidMessage) {

}