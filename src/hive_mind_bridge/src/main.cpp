#include "hivemind-bridge/Callback.h"
#include "hivemind-bridge/HiveMindBridge.h"
#include "ros/ros.h"
#include "swarmus_ros_navigation/MoveByMessage.h"
#include <cpp-common/ILogger.h>
#include <cstdarg>
#include <optional>
#include <pheromones/FunctionCallArgumentDTO.h>

constexpr uint8_t RATE_HZ{10};

class Logger : public ILogger {
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
            case LogLevel::Debug:
                ROS_DEBUG("%s", m_accumulatedString.c_str());
                break;
            case LogLevel::Info:
                ROS_INFO("%s", m_accumulatedString.c_str());
                break;
            case LogLevel::Warn:
                ROS_WARN("%s", m_accumulatedString.c_str());
                break;
            case LogLevel::Error:
                ROS_ERROR("%s", m_accumulatedString.c_str());
                break;
        }

        m_accumulatedString = "";
    }

private:
    std::string m_accumulatedString;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hive_mind_bridge");
    ros::NodeHandle nodeHandle("~");

    int port = nodeHandle.param("TCP_SERVER_PORT", 55551);
    std::string moveByTopic =
            nodeHandle.param("moveByTopic", std::string("/agent_1/navigation/moveBy"));
    ros::Publisher moveByPublisher =
            nodeHandle.advertise<swarmus_ros_navigation::MoveByMessage>(moveByTopic, 1000);
    Logger logger;
    HiveMindBridge bridge(port, logger);

    // Register custom actions
    CallbackFunction moveByCallback = [&](CallbackArgs args,
                                          int argsLength) -> std::optional<CallbackReturn> {
        swarmus_ros_navigation::MoveByMessage moveByMessage;

        auto* x = std::get_if<float>(&args[0].getArgument());
        auto* y = std::get_if<float>(&args[1].getArgument());

        if (x == nullptr || y == nullptr) {
            ROS_WARN("Received invalid argument type in moveby");
            return {};
        }

        moveByMessage.distance_x = *x;
        moveByMessage.distance_y = *y;

        // Publish on moveby
        moveByPublisher.publish(moveByMessage);
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



    CallbackFunction sendByteTo = [&](CallbackArgs args,
                                      int argsLength) -> std::optional<CallbackReturn> {
        swarmus_ros_navigation::MoveByMessage moveByMessage;

        auto* id = std::get_if<int64_t>(&args[0].getArgument());
        auto* byte = std::get_if<int64_t>(&args[1].getArgument());

        if (id == nullptr && byte == nullptr ) {
            ROS_WARN("Received invalid argument type in moveby");
            return {};
        }
        ROS_INFO("Sending byte: %d to: %d", byte[0], (uint32_t)*id);
        bridge.sendBytes((uint32_t)*id, (uint8_t*)byte, 1);
        return {};
    };
    CallbackArgsManifest sendByteToManifest;
    sendByteToManifest.push_back(
            UserCallbackArgumentDescription("id", FunctionDescriptionArgumentTypeDTO::Int));
    sendByteToManifest.push_back(
            UserCallbackArgumentDescription("byte", FunctionDescriptionArgumentTypeDTO::Int));
    bridge.registerCustomAction("sendByteTo", sendByteTo, sendByteToManifest);

    bridge.onBytesReceived([&](uint8_t* bytes, uint64_t bytesLength){
        for (uint i =0; i<bytesLength; i++){
            FunctionCallArgumentDTO args[1] {(int64_t)bytes[i]};
            FunctionCallRequestDTO fCall("setHex", args,1);
            UserCallRequestDTO UReq(UserCallTargetDTO::HOST, UserCallTargetDTO::BUZZ, fCall);
            RequestDTO req(69, UReq);
            MessageDTO msg(6, 1, req);
            ROS_INFO("Setting hex of: %d to: %d", 6, (int64_t)bytes[i]);
            bridge.queueAndSend(msg);
        }
    });

    // Register event hooks
    bridge.onConnect([]() { ROS_INFO("Client connected."); });

    bridge.onDisconnect([]() { ROS_INFO("Client disconnected."); });

    ros::Rate loopRate(RATE_HZ);
    while (ros::ok()) {
        ros::spinOnce();

        bridge.spin();

        loopRate.sleep();
    }

    return 0;
}
