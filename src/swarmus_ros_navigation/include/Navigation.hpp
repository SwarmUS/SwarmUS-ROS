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
#include <std_msgs/Float32.h>
#include <ros/ros.h>
#include <swarmus_ros_navigation/MoveByMessage.h>
#include <tf2_ros/transform_listener.h>

struct RosParameters {
    std::string tf_prefix;
    std::string moveBaseGlobalFrame;
};

class Navigation {
  protected:
    /**
     * @brief Stores the tf_prefix that needs to be added before each frame ID and stores the global
     *frame in wich command will be issue
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
     * @brief Subscriber used to listen to user commands
     **/
    ros::Subscriber m_RotateBySubscriber;

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
     * @brief Transform listener used to get transfrom between frames
     **/
    tf2_ros::TransformListener m_tfListener;

    /**
     * @brief Transform listener's buffer that holds the frames catch by the listener.
     **/
    tf2_ros::Buffer m_tfBuffer;

    /**
     * @brief Flag used to indicate if goal needs to be transform in the global frame of move_base.
     *Needed if the global frame is not base_footprint
     **/
    bool m_doGoalNeedsTransform;

    /**
     * @brief Id of the robot base_footprint
     **/
    std::string m_robotBaseFrame;

    /**
     * @brief Transforms the goal in the robot base frame into the frame of the move_base's global
     *costmap
     *
     * @param goalInBaseFrame Goal in the robot base frame
     *
     * @return Stamped pose of the goal in the move_base's global costmap
     *
     **/
    geometry_msgs::PoseStamped getGoalInGlobalFrame(
        const geometry_msgs::PoseStamped goalInBaseFrame);

    /**
     * @brief Transforms a x and y deplacement command from the user in a move_base goal
     *
     * @details Called whenever a command is sent by the user
     *
     * @param msg Message that contains a x and y deplacement command in meters
     *
     **/
    void moveByCallback(const swarmus_ros_navigation::MoveByMessage& msg);

    /**
     * @brief Rotate by a given angle in degrees
     *
     * @details Called whenever a command is sent by the user
     *
     * @param msg Message that contains the angle theta
     *
     **/
    void rotateByCallback(const std_msgs::Float32& msg);

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
     * @details Verifies if a new goal has been generated and, if it's the case, publishes the goal
     *to move_base
     *
     **/
    void execute();
};

#endif // NAVIGATION_HPP