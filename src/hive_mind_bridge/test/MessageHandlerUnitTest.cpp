#include "hive_mind_bridge/Callback.h"
#include "hive_mind_bridge/MessageHandler.h"
#include <gmock/gmock.h>
#include <hivemind-host/FunctionCallArgumentDTO.h>
#include <hivemind-host/FunctionCallRequestDTO.h>
#include <hivemind-host/FunctionDescriptionArgumentTypeDTO.h>
#include <hivemind-host/FunctionDescriptionRequestDTO.h>
#include <hivemind-host/MessageDTO.h>
#include <hivemind-host/RequestDTO.h>
#include <hivemind-host/UserCallRequestDTO.h>

class MessageHandlerFixture : public testing::Test {
  protected:
    bool m_testFunctionCalled = false;
    int m_testValue1 = 0;
    int m_testValue2 = 12;

    // Declare some test callbacks
    CallbackFunction m_testFunction = [&](CallbackArgs args,
                                          int argsLength) -> std::optional<CallbackReturn> {
        m_testFunctionCalled = true;

        return {};
    };

    CallbackFunction m_moveByTestCallback = [&](CallbackArgs args,
                                                int argsLength) -> std::optional<CallbackReturn> {
        m_testValue1 += std::get<int64_t>(args[0].getArgument());
        m_testValue2 -= std::get<float>(args[1].getArgument());

        return {};
    };
    CallbackArgsManifest m_moveByTestCallbackManifest;

    MessageHandler m_messageHandler;

    // Declare some test messages
    FunctionCallRequestDTO* m_functionCallRequestDto;
    UserCallRequestDTO* m_userCallRequestDto;
    RequestDTO* m_requestDto;
    MessageDTO* m_messageDto;

    FunctionCallRequestDTO* m_nonExistingFunctionCallRequestDto;
    UserCallRequestDTO* m_nonExistingUserCallRequestDto;
    RequestDTO* m_nonExistingRequestDto;
    MessageDTO* m_nonExistingMessageDto;

    FunctionCallArgumentDTO* m_sideEffectArg1;
    FunctionCallArgumentDTO* m_sideEffectArg2;
    FunctionCallRequestDTO* m_sideEffectFunctionCallRequestDto;
    UserCallRequestDTO* m_sideEffectUserCallRequestDto;
    RequestDTO* m_sideEffectRequestDto;

    MessageDTO* m_moveByMessageDto;

    void SetUp() override {
        m_moveByTestCallbackManifest.push_back(
            UserCallbackArgumentDescription("x", FunctionDescriptionArgumentTypeDTO::Int));
        m_moveByTestCallbackManifest.push_back(
            UserCallbackArgumentDescription("y", FunctionDescriptionArgumentTypeDTO::Float));
        m_messageHandler.registerCallback("MoveBy", m_moveByTestCallback,
                                          m_moveByTestCallbackManifest);

        // Existing void  function
        m_functionCallRequestDto =
            new FunctionCallRequestDTO("TestFunctionCallRequestDTO", nullptr, 0);
        m_userCallRequestDto = new UserCallRequestDTO(
            UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, *m_functionCallRequestDto);
        m_requestDto = new RequestDTO(1, *m_userCallRequestDto);
        m_messageDto = new MessageDTO(99, 2, *m_requestDto);

        // Nonexisting void function
        m_nonExistingFunctionCallRequestDto = new FunctionCallRequestDTO("NonExisting", nullptr, 0);
        m_nonExistingUserCallRequestDto = new UserCallRequestDTO(
            UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, *m_nonExistingFunctionCallRequestDto);
        m_nonExistingRequestDto = new RequestDTO(1, *m_nonExistingUserCallRequestDto);
        m_nonExistingMessageDto = new MessageDTO(99, 2, *m_nonExistingRequestDto);

        // Testing a callback with side effects
        m_sideEffectArg1 = new FunctionCallArgumentDTO((int64_t)1);
        m_sideEffectArg2 = new FunctionCallArgumentDTO((float)5);
        FunctionCallArgumentDTO args[2] = {*m_sideEffectArg1, *m_sideEffectArg2};
        m_sideEffectFunctionCallRequestDto = new FunctionCallRequestDTO("MoveBy", args, 2);
        m_sideEffectUserCallRequestDto = new UserCallRequestDTO(
            UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, *m_sideEffectFunctionCallRequestDto);
        m_sideEffectRequestDto = new RequestDTO(1, *m_sideEffectUserCallRequestDto);
        m_moveByMessageDto = new MessageDTO(99, 2, *m_sideEffectRequestDto);
    }

    void TearDown() override {
        delete m_functionCallRequestDto;
        delete m_userCallRequestDto;
        delete m_requestDto;
        delete m_messageDto;

        delete m_nonExistingFunctionCallRequestDto;
        delete m_nonExistingUserCallRequestDto;
        delete m_nonExistingRequestDto;
        delete m_nonExistingMessageDto;

        delete m_sideEffectArg1;
        delete m_sideEffectArg2;
        delete m_sideEffectFunctionCallRequestDto;
        delete m_sideEffectUserCallRequestDto;
        delete m_sideEffectRequestDto;
        delete m_moveByMessageDto;
    }
};

TEST_F(MessageHandlerFixture, registerNewCallBackSuccess) {
    ASSERT_FALSE(m_messageHandler.registerCallback("TestFunctionCallRequestDTO", m_testFunction));
}

TEST_F(MessageHandlerFixture, registerOverwriteCallbackSuccess) {
    m_messageHandler.registerCallback("TestFunctionCallRequestDTO", m_testFunction);
    ASSERT_TRUE(m_messageHandler.registerCallback("TestFunctionCallRequestDTO", m_testFunction));
}

TEST_F(MessageHandlerFixture, testGetCallbackSuccess) {
    m_messageHandler.registerCallback("TestFunctionCallRequestDTO", m_testFunction);
    ASSERT_TRUE(m_messageHandler.getCallback("TestFunctionCallRequestDTO"));
}

TEST_F(MessageHandlerFixture, testGetCallbackFail) {
    ASSERT_FALSE(m_messageHandler.getCallback("Nonexisting"));
}

TEST_F(MessageHandlerFixture, testHandleMessageVoidFunctionSuccess) {
    m_messageHandler.registerCallback("TestFunctionCallRequestDTO", m_testFunction);
    InboundRequestHandle result = std::get<InboundRequestHandle>(m_messageHandler.handleMessage(*m_messageDto));
    MessageDTO responseMessage = result.getResponse();

    ASSERT_EQ(responseMessage.getSourceId(), 2);
    ASSERT_EQ(responseMessage.getDestinationId(), 99);

    ResponseDTO response = std::get<ResponseDTO>(responseMessage.getMessage());
    ASSERT_EQ(response.getId(), 1);

    UserCallResponseDTO userCallResponse = std::get<UserCallResponseDTO>(response.getResponse());
    ASSERT_EQ(userCallResponse.getDestination(), UserCallTargetDTO::BUZZ);

    FunctionCallResponseDTO functionCallResponse =
        std::get<FunctionCallResponseDTO>(userCallResponse.getResponse());
    GenericResponseDTO genericResponse = functionCallResponse.getResponse();
    ASSERT_EQ(genericResponse.getStatus(), GenericResponseStatusDTO::Ok);

    result.getCallbackReturnContext().wait();

    ASSERT_TRUE(m_testFunctionCalled);
}

TEST_F(MessageHandlerFixture, TestHandleMessageMoveByFunctionSuccess) {
    InboundRequestHandle result = std::get<InboundRequestHandle>(m_messageHandler.handleMessage(*m_moveByMessageDto));
    MessageDTO responseMessage = result.getResponse();

    ASSERT_EQ(responseMessage.getSourceId(), 2);
    ASSERT_EQ(responseMessage.getDestinationId(), 99);

    ResponseDTO response = std::get<ResponseDTO>(responseMessage.getMessage());
    ASSERT_EQ(response.getId(), 1);

    UserCallResponseDTO userCallResponse = std::get<UserCallResponseDTO>(response.getResponse());
    ASSERT_EQ(userCallResponse.getDestination(), UserCallTargetDTO::BUZZ);

    FunctionCallResponseDTO functionCallResponse =
        std::get<FunctionCallResponseDTO>(userCallResponse.getResponse());
    GenericResponseDTO genericResponse = functionCallResponse.getResponse();
    ASSERT_EQ(genericResponse.getStatus(), GenericResponseStatusDTO::Ok);

    result.getCallbackReturnContext().wait();

    ASSERT_EQ(m_testValue1, 1);
    ASSERT_EQ(m_testValue2, 7);
}

TEST_F(MessageHandlerFixture, testHandleMessageFail) {
    InboundRequestHandle result = std::get<InboundRequestHandle>(m_messageHandler.handleMessage(*m_nonExistingMessageDto));
    MessageDTO responseMessage = result.getResponse();

    ASSERT_EQ(responseMessage.getSourceId(), 2);
    ASSERT_EQ(responseMessage.getDestinationId(), 99);

    ResponseDTO response = std::get<ResponseDTO>(responseMessage.getMessage());
    ASSERT_EQ(response.getId(), 1);

    UserCallResponseDTO userCallResponse = std::get<UserCallResponseDTO>(response.getResponse());
    ASSERT_EQ(userCallResponse.getDestination(), UserCallTargetDTO::BUZZ);

    FunctionCallResponseDTO functionCallResponse =
        std::get<FunctionCallResponseDTO>(userCallResponse.getResponse());
    GenericResponseDTO genericResponse = functionCallResponse.getResponse();
    ASSERT_EQ(genericResponse.getStatus(), GenericResponseStatusDTO::Unknown);
}

TEST_F(MessageHandlerFixture, handleFunctionListLengthRequest) {
    // Given
    FunctionListLengthRequestDTO functionListLengthRequest;
    UserCallRequestDTO userCallRequest(UserCallTargetDTO::UNKNOWN, UserCallTargetDTO::HOST,
                                       functionListLengthRequest);
    RequestDTO request(42, userCallRequest);
    MessageDTO incomingMessage(0, 0, request);

    m_messageHandler.registerCallback("TestFunctionCallRequestDTO1", m_testFunction);
    m_messageHandler.registerCallback("TestFunctionCallRequestDTO2", m_testFunction);
    m_messageHandler.registerCallback("TestFunctionCallRequestDTO3", m_testFunction);

    // When
    InboundRequestHandle result = std::get<InboundRequestHandle>(m_messageHandler.handleMessage(incomingMessage));

    // Then
    MessageDTO responseMessage = result.getResponse();
    ResponseDTO response = std::get<ResponseDTO>(responseMessage.getMessage());
    UserCallResponseDTO userCallResponse = std::get<UserCallResponseDTO>(response.getResponse());
    FunctionListLengthResponseDTO functionListLengthResponse =
        std::get<FunctionListLengthResponseDTO>(userCallResponse.getResponse());

    ASSERT_EQ(functionListLengthResponse.getLength(), 4);
}

TEST_F(MessageHandlerFixture, handleFunctionDescriptionRequest) {
    // Given
    FunctionDescriptionRequestDTO functionDescriptionRequest(0); // Testing 'MoveBy'
    UserCallRequestDTO userCallRequest(UserCallTargetDTO::UNKNOWN, UserCallTargetDTO::HOST,
                                       functionDescriptionRequest);
    RequestDTO request(42, userCallRequest);
    MessageDTO incomingMessage(0, 0, request);

    // When
    InboundRequestHandle result = std::get<InboundRequestHandle>(m_messageHandler.handleMessage(incomingMessage));

    // Then
    MessageDTO responseMessage = result.getResponse();
    ResponseDTO response = std::get<ResponseDTO>(responseMessage.getMessage());
    UserCallResponseDTO userCallResponse = std::get<UserCallResponseDTO>(response.getResponse());
    FunctionDescriptionResponseDTO functionDescriptionResponse =
        std::get<FunctionDescriptionResponseDTO>(userCallResponse.getResponse());
    FunctionDescriptionDTO functionDescription =
        std::get<FunctionDescriptionDTO>(functionDescriptionResponse.getResponse());

    ASSERT_EQ(functionDescription.getArgumentsLength(), 2);
    ASSERT_STREQ(functionDescription.getFunctionName(), "MoveBy");

    auto args = functionDescription.getArguments();
    ASSERT_STREQ(args[0].getArgumentName(), "x");
    ASSERT_EQ(args[0].getArgumentType(), FunctionDescriptionArgumentTypeDTO::Int);

    ASSERT_STREQ(args[1].getArgumentName(), "y");
    ASSERT_EQ(args[1].getArgumentType(), FunctionDescriptionArgumentTypeDTO::Float);
}

TEST_F(MessageHandlerFixture, handleFunctionDescriptionRequestVoid) {
    m_messageHandler.registerCallback("TestFunctionCallRequestDTO1", m_testFunction);

    // Given
    FunctionDescriptionRequestDTO functionDescriptionRequest(1); // Testing 'MoveBy'
    UserCallRequestDTO userCallRequest(UserCallTargetDTO::UNKNOWN, UserCallTargetDTO::HOST,
                                       functionDescriptionRequest);
    RequestDTO request(42, userCallRequest);
    MessageDTO incomingMessage(0, 0, request);

    // When
    InboundRequestHandle result = std::get<InboundRequestHandle>(m_messageHandler.handleMessage(incomingMessage));

    // Then
    MessageDTO responseMessage = result.getResponse();
    ResponseDTO response = std::get<ResponseDTO>(responseMessage.getMessage());
    UserCallResponseDTO userCallResponse = std::get<UserCallResponseDTO>(response.getResponse());
    FunctionDescriptionResponseDTO functionDescriptionResponse =
        std::get<FunctionDescriptionResponseDTO>(userCallResponse.getResponse());
    FunctionDescriptionDTO functionDescription =
        std::get<FunctionDescriptionDTO>(functionDescriptionResponse.getResponse());

    ASSERT_EQ(functionDescription.getArgumentsLength(), 0);
    ASSERT_STREQ(functionDescription.getFunctionName(), "TestFunctionCallRequestDTO1");
}

TEST_F(MessageHandlerFixture, handleFunctionDescriptionRequestOutOfBounds) {
    // Given
    FunctionDescriptionRequestDTO functionDescriptionRequest(99); // out of bounds!
    UserCallRequestDTO userCallRequest(UserCallTargetDTO::UNKNOWN, UserCallTargetDTO::HOST,
                                       functionDescriptionRequest);
    RequestDTO request(42, userCallRequest);
    MessageDTO incomingMessage(0, 0, request);

    // When
    InboundRequestHandle result = std::get<InboundRequestHandle>(m_messageHandler.handleMessage(incomingMessage));

    // Then
    MessageDTO responseMessage = result.getResponse();
    ResponseDTO response = std::get<ResponseDTO>(responseMessage.getMessage());
    UserCallResponseDTO userCallResponse = std::get<UserCallResponseDTO>(response.getResponse());

    FunctionCallResponseDTO functionCallResponse =
        std::get<FunctionCallResponseDTO>(userCallResponse.getResponse());
    GenericResponseDTO genericResponse = functionCallResponse.getResponse();
    ASSERT_EQ(genericResponse.getStatus(), GenericResponseStatusDTO::BadRequest);
    ASSERT_STREQ(genericResponse.getDetails(), "Index out of bounds.");
}

TEST_F(MessageHandlerFixture, handleInboundUserCallResponse) {
    // Given
    GenericResponseDTO genericResponse(GenericResponseStatusDTO::Ok, "All good");
    UserCallResponseDTO userCallResponse(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, genericResponse);
    ResponseDTO response(1, userCallResponse);
    MessageDTO incomingMessage(0, 0, response);

    // When
    InboundResponseHandle responseHandle = std::get<InboundResponseHandle>(m_messageHandler.handleMessage(incomingMessage));

    // Then
    ASSERT_EQ(responseHandle.getResponseId(), 1);
    ASSERT_EQ(responseHandle.getResponseStatus(), GenericResponseStatusDTO::Ok);
    ASSERT_STREQ(responseHandle.getStatusDetails().c_str(), "All good");
}

TEST_F(MessageHandlerFixture, handleInboundGenericResponse) {
    // Given
    GenericResponseDTO genericResponse(GenericResponseStatusDTO::Ok, "All good");
    ResponseDTO response(1, genericResponse);
    MessageDTO incomingMessage(0, 0, response);

    // When
    InboundResponseHandle responseHandle = std::get<InboundResponseHandle>(m_messageHandler.handleMessage(incomingMessage));

    // Then
    ASSERT_EQ(responseHandle.getResponseId(), 1);
    ASSERT_EQ(responseHandle.getResponseStatus(), GenericResponseStatusDTO::Ok);
    ASSERT_STREQ(responseHandle.getStatusDetails().c_str(), "All good");
}

TEST_F(MessageHandlerFixture, handleInboundFunctionResponse) {
    // Given
    GenericResponseDTO genericResponse(GenericResponseStatusDTO::Ok, "All good");
    FunctionCallResponseDTO functionCallResponse(genericResponse);
    UserCallResponseDTO userCallResponse(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, functionCallResponse);
    ResponseDTO response(1, userCallResponse);
    MessageDTO incomingMessage(0, 0, response);

    // When
    InboundResponseHandle responseHandle = std::get<InboundResponseHandle>(m_messageHandler.handleMessage(incomingMessage));

    // Then
    ASSERT_EQ(responseHandle.getResponseId(), 1);
    ASSERT_EQ(responseHandle.getResponseStatus(), GenericResponseStatusDTO::Ok);
    ASSERT_STREQ(responseHandle.getStatusDetails().c_str(), "All good");
}
