#include "Navigation.hpp"

#define RATE_HZ (10U)

/*************************************************************************************************/
Navigation::Navigation(std::shared_ptr<ros::NodeHandle> p_NodeHandle) {
    m_NodeHandle = p_NodeHandle;
    getRosParameters();
    std::string topic = "/" + m_RosParameters.robotName + "/navigation/moveBy";
    ROS_INFO("Subscribing to: %s", topic.c_str());
    m_MoveBySubscriber =
        m_NodeHandle->subscribe(topic.c_str(), 1000, &Navigation::moveByCallback, this);
    m_GoalPublisher = m_NodeHandle->advertise<geometry_msgs::PoseStamped>(
        m_RosParameters.clientDestination.c_str(), 1000);
}

/*************************************************************************************************/
void Navigation::getRosParameters() {
    if (ros::param::get("~robotName", m_RosParameters.robotName)) {
        m_RosParameters.clientDestination =
            '/' + m_RosParameters.robotName + "/move_base_simple/goal";
        ROS_INFO("Robot name provided: %s", m_RosParameters.robotName.c_str());
    } else {
        system("rosnode kill swarmus_ros_navigation_node"); // Node name defined launch file
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
    m_HasNewGoal = true;
}

/*************************************************************************************************/
void Navigation::execute() {
    ros::Rate loopRate(RATE_HZ);
    while (ros::ok()) {
        ros::spinOnce();
        if (m_HasNewGoal) {
            m_GoalPublisher.publish(m_CurrentGoal.target_pose);
            m_HasNewGoal = false;
        }
        loopRate.sleep();
    }
}