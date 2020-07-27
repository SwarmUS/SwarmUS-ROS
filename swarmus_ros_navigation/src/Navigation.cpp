#include "Navigation.hpp"

#define RATE_HZ (10U)

/*************************************************************************************************/
Navigation::Navigation(ros::NodeHandle* p_NodeHandle)  {
    m_NodeHandle = p_NodeHandle;
    getRosParameters();
    std::string topic = "/" + m_RosParameters.robot_name + "/navigation/moveBy";
    ROS_INFO("Subscribing to: %s", topic.c_str());
    m_MoveBySubscriber = m_NodeHandle->subscribe(topic.c_str(), 1000, &Navigation::moveByCallback, this);
}

/*************************************************************************************************/
void Navigation::getRosParameters(){
    if(ros::param::get("~robot_name", m_RosParameters.robot_name)) {
        m_RosParameters.clientDestination = '/' + m_RosParameters.robot_name + "/move_base";
        ROS_INFO("Robot name provided: %s", m_RosParameters.robot_name.c_str());
    }
    else {
        system("rosnode kill swarmus_ros_navigation_node"); // Node name defined launch file
    }

}


/*************************************************************************************************/
void Navigation::moveByCallback(const swarmus_ros_navigation::MoveByMessage& msg) {
    ROS_INFO("New goal received for robot: %s", m_RosParameters.robot_name.c_str());
    
    m_CurrentGoal.target_pose.header.frame_id = m_RosParameters.robot_name + "/base_footprint"; // relative position to bot

    m_CurrentGoal.target_pose.header.stamp = ros::Time::now();

    m_CurrentGoal.target_pose.pose.position.x = msg.distance_x;
    m_CurrentGoal.target_pose.pose.position.y = msg.distance_y;
    m_CurrentGoal.target_pose.pose.orientation.w = 0;
    m_HasNewGoal = true;
}

/*************************************************************************************************/
void Navigation::execute(){
    ros::Rate loopRate(RATE_HZ);
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> m_MoveBaseClient(m_RosParameters.clientDestination.c_str(), true);
    while(ros::ok()) {
        ros::spinOnce();
        if(m_HasNewGoal) {
            m_MoveBaseClient.sendGoal(m_CurrentGoal);
            m_HasNewGoal = false;
            m_MoveBaseClient.waitForResult();
        }       
        

        if(m_MoveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Current goal achieved");
        }            
        else {
            //ROS_ERROR("Goal failed");
        }
        loopRate.sleep();
    }
}