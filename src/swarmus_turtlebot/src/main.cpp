#include "hivemind-bridge/Callback.h"
#include "hivemind-bridge/HiveMindBridge.h"
#include <swarmus_turtlebot/Logger.h>
#include <pheromones/FunctionCallArgumentDTO.h>
#include "ros/ros.h"
#include "swarmus_turtlebot/Navigation.hpp"
#include <thread>
#include <cstdarg>
#include <optional>

static const uint8_t RATE_HZ{1};

static const uint32_t QUEUE_SIZE{1000};
const std::string ROBOT_BASE_FRAME{"base_footprint"};
const std::string MOVEBY_TOPIC{"navigation/moveBy"};
const std::string MOVEBASE_GOAL_TOPIC{"move_base_simple/goal"};

void navigationLoop(Navigation* navigation, ros::Rate loopRate) {
    while(true) {
        navigation->execute();

        loopRate.sleep();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "swarmus_turtlebot_bridge");
    std::shared_ptr<ros::NodeHandle> nodeHandle(new ros::NodeHandle(""));

    Navigation navigation(nodeHandle);

    ros::Rate loopRate(RATE_HZ);
    std::thread navigationThread(navigationLoop, &navigation, loopRate);

    int port = nodeHandle->param("TCP_SERVER_PORT", 55551);
    std::string moveByTopic =
            nodeHandle->param("moveByTopic", std::string("/navigation/moveBy"));
    ros::Publisher moveByPublisher =
            nodeHandle->advertise<swarmus_turtlebot::MoveByMessage>(moveByTopic, 1000);
    Logger logger;
    HiveMindBridge bridge(port, logger);

    // Register custom actions
    CallbackFunction moveByCallback = [&](CallbackArgs args,
                                          int argsLength) -> std::optional<CallbackReturn> {
        swarmus_turtlebot::MoveByMessage moveByMessage;

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

    // Register event hooks
    bridge.onConnect([]() { ROS_INFO("Client connected."); });

    bridge.onDisconnect([]() { ROS_INFO("Client disconnected."); });

    while (ros::ok()) {
        ros::spinOnce();
        bridge.spin();
        loopRate.sleep();
    }


    return 0;
}