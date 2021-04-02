#include "hive_mind_bridge/Callback.h"
#include "hive_mind_bridge/HiveMindBridge.h"
#include "hive_mind_bridge/MessageHandler.h"
#include "hive_mind_bridge/logger/ILogger.h"
#include "ros/ros.h"
#include "swarmus_ros_navigation/MoveByMessage.h"
#include <hivemind-host/FunctionCallArgumentDTO.h>
#include <optional>
#include <cstdarg>

constexpr uint8_t RATE_HZ{10};

class Logger : ILogger {
public:
    Logger() {}

    LogRet log(LogLevel level, const char* format, ...) override {
        va_list args;
        va_start(args, format);
        int retVal = formatAndAppend(format, args);
        va_end(args);

        flush(level);

        if (retVal >= 0) {
            return LogRet::Ok;
        } else {
            return LogRet::Error;
        }
    }

    int formatAndAppend(const char* format, va_list args) {
        // Copy varargs
        va_list vaCopy;
        va_copy(vaCopy, args);
        const int requiredLength = std::vsnprintf(NULL, 0, format, vaCopy);
        va_end(vaCopy);

        // Create a string with adequate length
        std::string tmpStr;
        tmpStr.resize((size_t)requiredLength);

        // Build a new string
        int retValue = vsnprintf(tmpStr.data(), tmpStr.size() + 1, format, args);
        m_accumulatedString = m_accumulatedString + tmpStr;

        return retValue;

    }

    void flush(LogLevel level) {
        switch (level) {
            case LogLevel::Debug :
                ROS_DEBUG("%s", m_accumulatedString.c_str());
                break;
            case LogLevel::Info :
                ROS_INFO("%s", m_accumulatedString.c_str());
                break;
            case LogLevel::Warn :
                ROS_WARN("%s", m_accumulatedString.c_str());
                break;
            case LogLevel::Error :
                ROS_ERROR("%s", m_accumulatedString.c_str());
                break;
        }

        m_accumulatedString = "";
    }

private:
    std::string m_accumulatedString;
};

int main(int argc, char** argv) {
    Logger logger;

    logger.log(LogLevel::Info, "Hello world %.2f", 12.0);

    /*
    ros::init(argc, argv, "hive_mind_bridge");
    ros::NodeHandle nodeHandle;

    std::string robotName = ros::param::param("~ROBOT_NAME", std::string("pioneer_0"));
    ros::Publisher moveByPublisher = nodeHandle.advertise<swarmus_ros_navigation::MoveByMessage>(
        robotName + "/navigation/moveBy", 1000);

    ros::Subscriber sub;

    int port = ros::param::param("~TCP_SERVER_PORT", 8080);
    HiveMindBridge bridge(port);

    // Register custom actions
    CallbackFunction moveByCallback = [&](CallbackArgs args,
                                          int argsLength) -> std::optional<CallbackReturn> {
        swarmus_ros_navigation::MoveByMessage moveByMessage;

        moveByMessage.distance_x = std::get<float>(args[0].getArgument());
        moveByMessage.distance_y = std::get<float>(args[1].getArgument());

        // Publish on moveby
        moveByPublisher.publish(moveByMessage);

        std::this_thread::sleep_for(
            std::chrono::seconds(2)); // Just to show that callbacks can be blocking

        return {};
    };

    CallbackArgsManifest moveByManifest;
    moveByManifest.push_back(
        UserCallbackArgumentDescription("x", FunctionDescriptionArgumentTypeDTO::Float));
    moveByManifest.push_back(
        UserCallbackArgumentDescription("y", FunctionDescriptionArgumentTypeDTO::Float));
    bridge.registerCustomAction("moveBy", moveByCallback, moveByManifest);

    CallbackFunction getStatus = [&](CallbackArgs args,
                                     int argsLength) -> std::optional<CallbackReturn> {
        // todo This remains to be implemented.
        int64_t isRobotOk = 1;

        CallbackArgs returnArgs;
        returnArgs[0] = FunctionCallArgumentDTO(isRobotOk);

        CallbackReturn cbReturn("getStatusReturn", returnArgs);

        return cbReturn;
    };
    bridge.registerCustomAction("getStatus", getStatus);

    // Register event hooks
    bridge.onConnect([]() { ROS_INFO("Client connected."); });

    bridge.onDisconnect([]() { ROS_INFO("Client disconnected."); });

    ros::Rate loopRate(RATE_HZ);
    while (ros::ok()) {
        ros::spinOnce();

        bridge.spin();

        loopRate.sleep();
    }
    */

    return 0;
}