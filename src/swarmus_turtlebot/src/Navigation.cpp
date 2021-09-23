#include "swarmus_turtlebot/Navigation.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static const uint32_t QUEUE_SIZE{1000};
const std::string ROBOT_BASE_FRAME{"base_footprint"};
const std::string MOVEBY_TOPIC{"navigation/moveBy"};
const std::string MOVEBASE_GOAL_TOPIC{"move_base_simple/goal"};

/*************************************************************************************************/
Navigation::Navigation(std::shared_ptr<ros::NodeHandle> p_NodeHandle) : m_tfListener(m_tfBuffer) {
    if (p_NodeHandle) {
        m_NodeHandle = p_NodeHandle;

        fetchRosParameters();

        ROS_INFO("Subscribing to: %s",
                 (ros::this_node::getNamespace() + "/" + MOVEBY_TOPIC).c_str());
        m_MoveBySubscriber =
            m_NodeHandle->subscribe(MOVEBY_TOPIC, QUEUE_SIZE, &Navigation::moveByCallback, this);

        m_GoalPublisher =
            m_NodeHandle->advertise<geometry_msgs::PoseStamped>(MOVEBASE_GOAL_TOPIC, QUEUE_SIZE);
    }
}

Navigation::~Navigation() { return; }

/*************************************************************************************************/
void Navigation::fetchRosParameters() {
    if (ros::param::get("~tf_prefix", m_RosParameters.tf_prefix)) {
        ROS_INFO("tf_prefix provided: %s", m_RosParameters.tf_prefix.c_str());

        m_robotBaseFrame = m_RosParameters.tf_prefix + "/" + ROBOT_BASE_FRAME;

        const std::string moveBaseGlobalFrameParam = "move_base/global_costmap/global_frame";
        if (ros::param::get(moveBaseGlobalFrameParam, m_RosParameters.moveBaseGlobalFrame)) {
            // TODO add a check to see if a transform exist between frames
            m_doGoalNeedsTransform = m_RosParameters.moveBaseGlobalFrame != m_robotBaseFrame;
        } else {
            ROS_WARN("Couldn't get the global frame of the move_base's global costmap. The "
                     "swarmus_turtlebot node will publish goal in the %s frame",
                     m_robotBaseFrame.c_str());
            m_doGoalNeedsTransform = false;
        }
    } else {
        m_robotBaseFrame = ROBOT_BASE_FRAME;
        m_doGoalNeedsTransform = false;
        ROS_WARN("No tf_prefix was provided. The swarmus_turtlebot node will publish goal in "
                 "the %s frame",
                 m_robotBaseFrame.c_str());
    }
}

/*************************************************************************************************/
void Navigation::moveByCallback(const swarmus_turtlebot::MoveBy& msg) {
    // Generate the goal pose in the robot base frame
    geometry_msgs::PoseStamped goalPose;

    goalPose.header.stamp = ros::Time::now();

    goalPose.pose.position.x = msg.distance_x;
    goalPose.pose.position.y = msg.distance_y;
    goalPose.pose.position.z = 0;

    goalPose.pose.orientation.x = 0;
    goalPose.pose.orientation.y = 0;
    goalPose.pose.orientation.z = 0;
    goalPose.pose.orientation.w = 1;

    // Transform the goal in the global frame if needed
    if (m_doGoalNeedsTransform) {
        goalPose = getGoalInGlobalFrame(goalPose);
        goalPose.header.frame_id = m_RosParameters.moveBaseGlobalFrame;
    } else {
        goalPose.header.frame_id = m_robotBaseFrame;
    }

    // Update the current goal
    m_CurrentGoal.target_pose = goalPose;
    m_hasNewGoal = true;
}

geometry_msgs::PoseStamped Navigation::getGoalInGlobalFrame(
    const geometry_msgs::PoseStamped goalInBaseFrame) {

    geometry_msgs::TransformStamped robotBaseToGlobalTransform;
    geometry_msgs::PoseStamped goalInGlobalFrame;

    try {
        robotBaseToGlobalTransform = m_tfBuffer.lookupTransform(m_RosParameters.moveBaseGlobalFrame,
                                                                m_robotBaseFrame, ros::Time(0));

        tf2::doTransform(goalInBaseFrame, goalInGlobalFrame, robotBaseToGlobalTransform);
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Navigation: %s", ex.what());
    }

    return goalInGlobalFrame;
}

/*************************************************************************************************/
void Navigation::execute() {
    if (m_hasNewGoal) {
        m_GoalPublisher.publish(m_CurrentGoal.target_pose);
        m_hasNewGoal = false;
    }
}