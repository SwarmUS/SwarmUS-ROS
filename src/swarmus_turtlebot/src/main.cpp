#include "ros/ros.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

static const uint8_t RATE_HZ{1};

static const uint32_t QUEUE_SIZE{1000};
const std::string ROBOT_BASE_FRAME{"base_footprint"};
const std::string MOVEBY_TOPIC{"navigation/moveBy"};
const std::string MOVEBASE_GOAL_TOPIC{"move_base_simple/goal"};

int main(int argc, char** argv) {
    ros::init(argc, argv, "swarmus_turtlebot_bridge");

    std::shared_ptr<ros::NodeHandle> nodeHandle(new ros::NodeHandle(""));

    ros::Publisher goalPublisher =
            nodeHandle->advertise<geometry_msgs::PoseStamped>(MOVEBASE_GOAL_TOPIC, QUEUE_SIZE);

    ros::Rate loopRate(RATE_HZ);

    while (ros::ok()) {
        geometry_msgs::PoseStamped goalPose;

        goalPose.header.stamp = ros::Time::now();

        goalPose.pose.position.x = 0;
        goalPose.pose.position.y = 1;
        goalPose.pose.position.z = 0;

        goalPose.pose.orientation.x = 0;
        goalPose.pose.orientation.y = 0;
        goalPose.pose.orientation.z = 0;
        goalPose.pose.orientation.w = 1;

        goalPose.header.frame_id = ROBOT_BASE_FRAME;

        goalPublisher.publish(goalPose);


        ros::spinOnce();
        loopRate.sleep();
    }


    return 0;
}