#ifndef BUZZBRIDGE_HPP
#define BUZZBRIDGE_HPP

#include "BuzzUtility.hpp"
#include <ros/ros.h>
#include <swarmus_ros_simulation/InterLocalization_grid.h>
#include <std_msgs/String.h>
#include <swarmus_ros_navigation/MoveByMessage.h>

#define BUZZRATE 10 // Frequency desired for buzz (in Hz)

struct BuzzFiles_t {
    std::string script;
    std::string byteCode;
    std::string debugCode;
};

struct RosParameters_t {
    std::string robot_name;
    int robotID;
    BuzzFiles_t bzzFileName;
};

class BuzzBridge {
protected:
    void getROSParameters();
    void registerSubcriberCallbacks();

    void interlocGridCallback(const swarmus_ros_simulation::InterLocalization_grid &p_Grid);
    void interCommunicationCallback(const std_msgs::String::ConstPtr& msg);

    int moveByClosure(buzzvm_t vm = BuzzUtility::getVM());

    RosParameters_t m_RosParameters;

    ros::NodeHandle* m_NodeHandle;
    ros::Subscriber m_InterlocalisationSubscriber;
    ros::Subscriber m_IntercommunicationSubscriber;
    ros::Publisher m_MoveByPublisher;
public:
    BuzzBridge(ros::NodeHandle* p_NodeHandle);
    ~BuzzBridge();
    void execute(void);
    void registerHookFunctions();

};




#endif // BUZZBRIDGE_HPP