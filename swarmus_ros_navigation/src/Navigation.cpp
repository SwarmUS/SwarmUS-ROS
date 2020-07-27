#include "Navigation.hpp"

#define RATE_HZ (10U)

/*************************************************************************************************/
Navigation::Navigation(ros::NodeHandle* p_NodeHandle) : m_MoveBaseClient("move_base", true) {
    m_NodeHandle = p_NodeHandle;
    getRosParameters();

    std::string topic = "/" + m_RosParameters.robot_name + "navigation/moveBy";
    m_MoveBySubscriber = m_NodeHandle->subscribe(topic.c_str(), 1000, &Navigation::moveByCallback, this);
}

/*************************************************************************************************/
void Navigation::getRosParameters(){
    if(ros::param::get("~robot_name", m_RosParameters.robot_name)) {
        ROS_INFO("Robot name provided: %s", m_RosParameters.robot_name.c_str());
    }
    else {
        system("rosnode kill swarmus_ros_navigation_node"); // Node name defined launch file
    }

}


/*************************************************************************************************/
void Navigation::moveByCallback(const swarmus_ros_navigation::MoveByMessage& msg) {
    m_CurrentGoal.target_pose.header.frame_id = m_RosParameters.robot_name;
    m_CurrentGoal.target_pose.header.stamp = ros::Time::now();

    m_CurrentGoal.target_pose.pose.position.x = msg.distance_x;
    m_CurrentGoal.target_pose.pose.orientation.w = msg.distance_y;
    m_HasNewGoal = true;
}

/*************************************************************************************************/
void Navigation::execute(){
    ros::Rate loopRate(RATE_HZ);
    while(ros::ok()) {
        if(m_HasNewGoal) {
            m_MoveBaseClient.sendGoal(m_CurrentGoal);
            m_HasNewGoal = false;
        }

        ros::spinOnce();
        m_MoveBaseClient.waitForResult();

        if(m_MoveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Current goal achieved");
        }            
        else {
            ROS_ERROR("Goal failed");
        }
        loopRate.sleep();
    }
}