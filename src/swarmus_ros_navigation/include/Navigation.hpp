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
    /**
    * @brief Stores the robotName from the get robot_name ROS param and generates the clientDestination which is the name of the move_base topic
    **/
    void fetchRosParameters();

    /**
    * @brief Struct of paramaters used by the subscriber and publisher
    **/
    RosParameters m_RosParameters;

    /**
    * @brief Shared pointer of the NodeHandle
    **/
    std::shared_ptr<ros::NodeHandle> m_NodeHandle;

    /**
    * @brief Subscriber used to listen to user commands
    **/
    ros::Subscriber m_MoveBySubscriber;

    /**
    * @brief Publisher used to send goal commands to move_base
    **/
    ros::Publisher m_GoalPublisher;
    
    /**
    * @brief Current move_base goal of the robot
    **/
    move_base_msgs::MoveBaseGoal m_CurrentGoal;

    /**
    * @brief Flag used to indicate when a new goal is generated
    **/
    bool m_hasNewGoal;

    /**
    * @brief Transforms a x and y deplacement command from the user in a move_base goal
    * 
    * @details Called whenever a command is sent by the user
    * 
    * @param msg Message that contains a x and y deplacement command in meters
    * 
    **/
    void moveByCallback(const swarmus_ros_navigation::MoveByMessage& msg);

  public:
    /**
    * @brief Constructor
    * 
    * @param nodeHandle Shared pointer of the handle of the node
    * 
    **/
    Navigation(std::shared_ptr<ros::NodeHandle> nodeHandle);

    /**
    * @brief Destructor
    **/
    ~Navigation();

    /**
    * @brief Reads the state of the object and sends commands accordingly.
    * 
    * @details Verifies if a new goal has been generated and, if it's the case, publishes the goal to move_base
    * 
    **/
    void execute();
};

#endif // NAVIGATION_HPP