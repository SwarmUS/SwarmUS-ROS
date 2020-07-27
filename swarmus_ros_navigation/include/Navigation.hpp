/******************************************
 * Author: SwarmUS
 * Description: Contains a node description
 * to accept various commands as message to
 * create a Navigation stack.
 ******************************************/
#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <swarmus_ros_navigation/MoveByMessage.h>
#include <iostream>
#include <array>

struct RosParameters_t {
    std::string robot_name;
};


class Navigation {
protected:
    void getRosParameters();
    RosParameters_t m_RosParameters;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> m_MoveBaseClient;
    
    ros::NodeHandle* m_NodeHandle;
    ros::Subscriber m_MoveBySubscriber;

    move_base_msgs::MoveBaseGoal m_CurrentGoal;

    bool m_HasNewGoal;

    void moveByCallback(const swarmus_ros_navigation::MoveByMessage& msg);
public:
    Navigation(ros::NodeHandle* p_NodeHandle);

    void execute();
    
};


#endif // NAVIGATION_HPP