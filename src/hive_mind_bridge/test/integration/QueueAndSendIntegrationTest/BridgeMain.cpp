#include "../../utils/Logger.h"
#include "hive_mind_bridge/HiveMindBridge.h"
#include "hive_mind_bridge/MessageHandler.h"
#include <cpp-common/ILogger.h>

int main(int argc, char** argv) {
    int port = 8080;
    Logger logger;
    HiveMindBridge bridge(port, logger);

    // Register event hooks
    bridge.onConnect([&]() { logger.log(LogLevel::Info, "Client connected."); });

    bridge.onDisconnect([&]() { logger.log(LogLevel::Info, "Client disconnected."); });

    while (true) {

        bridge.queueAndSend(MessageUtils::createFunctionCallRequest(
            42, 42, 1, UserCallTargetDTO::UNKNOWN, "someRemoteCallback"));

        bridge.spin();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}