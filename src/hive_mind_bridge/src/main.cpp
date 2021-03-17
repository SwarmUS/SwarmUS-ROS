#include "hive_mind_bridge/HiveMindBridge.h"
#include "hive_mind_bridge/MessageHandler.h"
#include "ros/ros.h"
#include "swarmus_ros_navigation/MoveByMessage.h"
#include <hivemind-host/FunctionCallArgumentDTO.h>
#include <optional>

constexpr uint8_t RATE_HZ{10};
constexpr uint32_t compoundId{1}; // TODO find a way for the HiveMind and the robot to share this ID

int main(int argc, char** argv) {
    ros::init(argc, argv, "hive_mind_bridge");
    ros::NodeHandle nodeHandle;

    std::string robotName = ros::param::param("~ROBOT_NAME", std::string("pioneer_0"));
    ros::Publisher moveByPublisher = nodeHandle.advertise<swarmus_ros_navigation::MoveByMessage>(
        robotName + "/navigation/moveBy", 1000);

    ros::Subscriber sub;

    int port = ros::param::param("~TCP_SERVER_PORT", 8080);
    HiveMindBridge bridge(port);

    // Register custom actions
    CallbackFunction moveByCallback = [&](CallbackArgs args, int argsLength) -> CallbackArgs {
        swarmus_ros_navigation::MoveByMessage moveByMessage;

        moveByMessage.distance_x = std::get<float>(args[0].getArgument());
        moveByMessage.distance_y = std::get<float>(args[1].getArgument());

        // Publish on moveby
        moveByPublisher.publish(moveByMessage);

        std::this_thread::sleep_for(std::chrono::seconds (2));

        ROS_INFO("CALLBACK WILL RETURN JUST NOW");

        return {};
    };

    CallbackArgsManifest moveByManifest;
    moveByManifest.push_back(
        UserCallbackArgumentDescription("x", FunctionDescriptionArgumentTypeDTO::Float));
    moveByManifest.push_back(
        UserCallbackArgumentDescription("y", FunctionDescriptionArgumentTypeDTO::Float));
    bridge.registerCustomAction("moveBy", moveByCallback, moveByManifest);

    CallbackFunction getPosition = [&](CallbackArgs args, int argsLength) -> CallbackArgs  {
        // Go get the values elsewhere...
        int64_t x = 1;
        float y = 2.0;

        CallbackArgs returnArgs;
        returnArgs[1] = FunctionCallArgumentDTO(x);
        returnArgs[2] = FunctionCallArgumentDTO(y);

        return returnArgs;
    };
    bridge.registerCustomAction("getPosition", getPosition);

    CallbackFunction getPositionReturn = [&](CallbackArgs args, int argsLength) -> CallbackArgs  {
        // What to do when we receive a position.
        ROS_INFO("RECEIVED A POSITION: (%d, %f)", std::get<int64_t>(args[0].getArgument()), std::get<float>(args[1].getArgument()));

        return {};
    };

    CallbackArgsManifest  getPositionReturnManifest;
    getPositionReturnManifest.push_back(UserCallbackArgumentDescription("x", FunctionDescriptionArgumentTypeDTO::Int));
    getPositionReturnManifest.push_back(UserCallbackArgumentDescription("y", FunctionDescriptionArgumentTypeDTO::Float));
    bridge.registerCustomAction("getPositionReturn", getPositionReturn, getPositionReturnManifest);

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