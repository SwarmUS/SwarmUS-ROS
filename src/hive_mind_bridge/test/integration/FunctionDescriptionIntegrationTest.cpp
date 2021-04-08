#include <pheromones/FunctionListLengthRequestDTO.h>
#include <pheromones/UserCallRequestDTO.h>
#include <pheromones/MessageDTO.h>
#include <pheromones/RequestDTO.h>
#include <pheromones/HiveMindHostDeserializer.h>
#include <pheromones/HiveMindHostSerializer.h>
#include <thread>
#include "../utils/TCPClient.h"
#include "../utils/Logger.h"
#include <gmock/gmock.h>
#include "hive_mind_bridge/HiveMindBridge.h"

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
        CallbackFunction moveByCallback = [&](CallbackArgs args,
                                              int argsLength) -> std::optional<CallbackReturn> {

            return {};
        };

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

    void SetUp() {
    }

    void TearDown() {
    };

public:
    FunctionDescriptionRequestIntegrationTestFixture() {
        // Bridge side
        m_bridge = new HiveMindBridge(m_tcpPort, m_logger);
        m_bridgeThread = std::thread(&FunctionDescriptionRequestIntegrationTestFixture::bridgeThread, this);

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
    for (int i = 0; i < 2; i++) {
        // Given
        FunctionListLengthRequestDTO listLengthRequest;
        UserCallRequestDTO userCallRequest(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, listLengthRequest);
        RequestDTO request(1, userCallRequest);
        MessageDTO requestMessage(1, 1, request);

        // When
        m_clientSerializer->serializeToStream(requestMessage);

        MessageDTO responseMessage;
        m_clientDeserializer->deserializeFromStream(responseMessage);

        // Then
        auto response = std::get<ResponseDTO>(responseMessage.getMessage());
        auto userCallResponse = std::get<UserCallResponseDTO>(response.getResponse());
        auto functionListLengthResponse = std::get<FunctionListLengthResponseDTO>(userCallResponse.getResponse());

        ASSERT_EQ(functionListLengthResponse.getLength(), 2);
    }
}

TEST_F(FunctionDescriptionRequestIntegrationTestFixture, testFunctionDescriptionRequest) {
    // Given
    FunctionDescriptionRequestDTO functionDescriptionRequest(0);
    UserCallRequestDTO userCallRequest(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, functionDescriptionRequest);
    RequestDTO request(1, userCallRequest);
    MessageDTO requestMessage(1, 1, request);

    // When
    m_clientSerializer->serializeToStream(requestMessage);

    MessageDTO responseMessage;
    m_clientDeserializer->deserializeFromStream(responseMessage);

    // Then
    auto response = std::get<ResponseDTO>(responseMessage.getMessage());
    auto userCallResponse = std::get<UserCallResponseDTO>(response.getResponse());
    auto functionDescriptionResponse = std::get<FunctionDescriptionResponseDTO>(userCallResponse.getResponse());
    auto functionDescription = std::get<FunctionDescriptionDTO>(functionDescriptionResponse.getResponse());

    ASSERT_STREQ("moveBy", functionDescription.getFunctionName());
}
