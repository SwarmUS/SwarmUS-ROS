#include "../utils/Logger.h"
#include "../utils/TCPClient.h"
#include "hive_mind_bridge/HiveMindBridge.h"
#include <gmock/gmock.h>
#include <pheromones/FunctionListLengthRequestDTO.h>
#include <pheromones/HiveMindHostDeserializer.h>
#include <pheromones/HiveMindHostSerializer.h>
#include <pheromones/MessageDTO.h>
#include <pheromones/RequestDTO.h>
#include <pheromones/UserCallRequestDTO.h>
#include <thread>

bool g_threadShouldRun = true;

class FunctionDescriptionRequestIntegrationTestFixture : public testing::Test {
  protected:
    Logger m_logger;
    int m_tcpPort = 5001;

    // Bridge side
    HiveMindBridge* m_bridge;

    // Client side
    TCPClient* m_tcpClient;
    HiveMindHostSerializer* m_clientSerializer;
    HiveMindHostDeserializer* m_clientDeserializer;

    void setUpCallbacks() {
        // Register custom actions
        CallbackFunction moveByCallback =
            [&](CallbackArgs args, int argsLength) -> std::optional<CallbackReturn> { return {}; };

        CallbackArgsManifest moveByManifest;
        moveByManifest.push_back(
            UserCallbackArgumentDescription("x", FunctionDescriptionArgumentTypeDTO::Float));
        moveByManifest.push_back(
            UserCallbackArgumentDescription("y", FunctionDescriptionArgumentTypeDTO::Float));
        m_bridge->registerCustomAction("moveBy", moveByCallback, moveByManifest);

        CallbackFunction getStatus = [&](CallbackArgs args,
                                         int argsLength) -> std::optional<CallbackReturn> {
            int64_t isRobotOk = 1;

            CallbackArgs returnArgs;
            returnArgs[0] = FunctionCallArgumentDTO(isRobotOk);

            CallbackReturn cbReturn("getStatusReturn", returnArgs);

            return cbReturn;
        };
        m_bridge->registerCustomAction("getStatus", getStatus);
    }

    void connectClient() {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        m_tcpClient->connect();
    }

    void greet() {
        // Wait for a greet message
        MessageDTO greetRequest;
        if (m_clientDeserializer->deserializeFromStream(greetRequest)) {
            // The client has ID 42
            MessageDTO greetResponse(42, 42, GreetingDTO(42));
            m_clientSerializer->serializeToStream(greetResponse);
        } else {
            m_logger.log(LogLevel::Warn, "Deserializing greet failed.");
        }
    }

    void SetUp() { std::this_thread::sleep_for(std::chrono::milliseconds(250)); }

    void TearDown(){};

  public:
    FunctionDescriptionRequestIntegrationTestFixture() {
        // Bridge side
        m_bridge = new HiveMindBridge(m_tcpPort, m_logger);
        m_bridgeThread =
            std::thread(&FunctionDescriptionRequestIntegrationTestFixture::bridgeThread, this);

        setUpCallbacks();

        // Client side
        m_tcpClient = new TCPClient(m_tcpPort);
        m_clientSerializer = new HiveMindHostSerializer(*m_tcpClient);
        m_clientDeserializer = new HiveMindHostDeserializer(*m_tcpClient);

        connectClient();

        greet();
    }

    ~FunctionDescriptionRequestIntegrationTestFixture() {
        g_threadShouldRun = false;
        m_bridgeThread.join();

        delete m_tcpClient;
        delete m_clientSerializer;
        delete m_clientDeserializer;
        delete m_bridge;
    }

    std::thread m_bridgeThread;

    void bridgeThread() {
        while (g_threadShouldRun) {
            m_bridge->spin();
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
    }
};

TEST_F(FunctionDescriptionRequestIntegrationTestFixture, testFunctionListLengthRequest) {
    for (int i = 0; i < 5; i++) {
        // LIST LENGTH
        // Given
        FunctionListLengthRequestDTO listLengthRequest;
        UserCallRequestDTO userCallRequest(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST,
                                           listLengthRequest);
        RequestDTO request(1, userCallRequest);
        MessageDTO requestMessage(1, 1, request);

        // When
        m_clientSerializer->serializeToStream(requestMessage);

        MessageDTO responseMessage;
        m_clientDeserializer->deserializeFromStream(responseMessage);

        // Then
        auto response = std::get<ResponseDTO>(responseMessage.getMessage());
        auto userCallResponse = std::get<UserCallResponseDTO>(response.getResponse());
        auto functionListLengthResponse =
            std::get<FunctionListLengthResponseDTO>(userCallResponse.getResponse());

        ASSERT_EQ(functionListLengthResponse.getLength(), 2);

        // FUNCTION DESCRIPTION 0

        // Given
        FunctionDescriptionRequestDTO functionDescriptionRequest(0);
        UserCallRequestDTO descriptionUserCallRequest(
            UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, functionDescriptionRequest);
        RequestDTO descriptionRequest(1, descriptionUserCallRequest);
        MessageDTO descriptionRequestMessage(1, 1, descriptionRequest);

        // When
        m_clientSerializer->serializeToStream(descriptionRequestMessage);

        MessageDTO descriptionResponseMessage;
        m_clientDeserializer->deserializeFromStream(descriptionResponseMessage);

        // Then
        auto descriptionResponse = std::get<ResponseDTO>(descriptionResponseMessage.getMessage());
        auto descriptionUserCallResponse =
            std::get<UserCallResponseDTO>(descriptionResponse.getResponse());
        auto functionDescriptionResponse =
            std::get<FunctionDescriptionResponseDTO>(descriptionUserCallResponse.getResponse());
        auto functionDescription =
            std::get<FunctionDescriptionDTO>(functionDescriptionResponse.getResponse());

        ASSERT_STREQ(functionDescription.getFunctionName(), "moveBy");
        ASSERT_EQ(functionDescription.getArgumentsLength(), 2);

        // FUNCTION DESCRIPTION 1
        // Given
        FunctionDescriptionRequestDTO functionDescriptionRequest_1(1);
        UserCallRequestDTO descriptionUserCallRequest_1(
            UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, functionDescriptionRequest_1);
        RequestDTO descriptionRequest_1(1, descriptionUserCallRequest_1);
        MessageDTO descriptionRequestMessage_1(1, 1, descriptionRequest_1);

        // When
        m_clientSerializer->serializeToStream(descriptionRequestMessage_1);

        MessageDTO descriptionResponseMessage_1;
        m_clientDeserializer->deserializeFromStream(descriptionResponseMessage_1);

        // Then
        auto descriptionResponse_1 =
            std::get<ResponseDTO>(descriptionResponseMessage_1.getMessage());
        auto descriptionUserCallResponse_1 =
            std::get<UserCallResponseDTO>(descriptionResponse_1.getResponse());
        auto functionDescriptionResponse_1 =
            std::get<FunctionDescriptionResponseDTO>(descriptionUserCallResponse_1.getResponse());
        auto functionDescription_1 =
            std::get<FunctionDescriptionDTO>(functionDescriptionResponse_1.getResponse());

        ASSERT_STREQ(functionDescription_1.getFunctionName(), "getStatus");
        ASSERT_EQ(functionDescription_1.getArgumentsLength(), 0);
    }
}
