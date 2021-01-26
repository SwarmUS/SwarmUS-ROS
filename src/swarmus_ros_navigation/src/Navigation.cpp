#include "Navigation.hpp"

static const uint32_t QUEUE_SIZE{1000};

/*************************************************************************************************/
Navigation::Navigation(std::shared_ptr<ros::NodeHandle> p_NodeHandle) {
    if (p_NodeHandle){
        m_NodeHandle = p_NodeHandle;
        fetchRosParameters();
        std::string topic = "/" + m_RosParameters.robotName + "/navigation/moveBy";
        ROS_INFO("Subscribing to: %s", topic.c_str());
        m_MoveBySubscriber =
            m_NodeHandle->subscribe(topic, QUEUE_SIZE, &Navigation::moveByCallback, this);
        m_GoalPublisher = m_NodeHandle->advertise<geometry_msgs::PoseStamped>(
            m_RosParameters.clientDestination, QUEUE_SIZE);
    }
}

Navigation::~Navigation() {
    return;
}

/*************************************************************************************************/
void Navigation::fetchRosParameters() {
    if (ros::param::get("~robot_name", m_RosParameters.robotName)) {
        m_RosParameters.clientDestination =
            '/' + m_RosParameters.robotName + "/move_base_simple/goal";
        ROS_INFO("Robot name provided: %s", m_RosParameters.robotName.c_str());
    } else {
        ROS_ERROR("robot_name is not defined");
    }
}

/*************************************************************************************************/
void Navigation::moveByCallback(const swarmus_ros_navigation::MoveByMessage& msg) {
    ROS_INFO("New goal received for robot: %s", m_RosParameters.robotName.c_str());

    m_CurrentGoal.target_pose.header.frame_id =
        m_RosParameters.robotName + "/base_footprint"; // relative position to bot

    m_CurrentGoal.target_pose.header.stamp = ros::Time::now();

    m_CurrentGoal.target_pose.pose.position.x = msg.distance_x;
    m_CurrentGoal.target_pose.pose.position.y = msg.distance_y;
    m_CurrentGoal.target_pose.pose.position.z = 0;

    m_CurrentGoal.target_pose.pose.orientation.x = 0;
    m_CurrentGoal.target_pose.pose.orientation.y = 0;
    m_CurrentGoal.target_pose.pose.orientation.z = 0;
    m_CurrentGoal.target_pose.pose.orientation.w = 1;
    m_hasNewGoal = true;
}

/*************************************************************************************************/
void Navigation::execute() {
    if (m_hasNewGoal) {
        m_GoalPublisher.publish(m_CurrentGoal.target_pose);
        m_hasNewGoal = false;
    }
}