/******************************************
 * Author: SwarmUS
 * Description: Contains a node description
 * to accept various commands as message to
 * create a Navigation stack.
 ******************************************/
#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include <actionlib/client/simple_action_client.h>
#include <array>
#include <iostream>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <swarmus_ros_navigation/MoveByMessage.h>

struct RosParameters {
    std::string robotName;
    std::string clientDestination;
};

class Navigation {
  protected:
    void fetchRosParameters();
    RosParameters m_RosParameters;

    std::shared_ptr<ros::NodeHandle> m_NodeHandle;
    ros::Subscriber m_MoveBySubscriber;
    ros::Publisher m_GoalPublisher;
    move_base_msgs::MoveBaseGoal m_CurrentGoal;

    bool m_hasNewGoal;

    void moveByCallback(const swarmus_ros_navigation::MoveByMessage& msg);

  public:
    Navigation(std::shared_ptr<ros::NodeHandle> p_NodeHandle);
    ~Navigation();

    void execute();
};

#endif // NAVIGATION_HPP