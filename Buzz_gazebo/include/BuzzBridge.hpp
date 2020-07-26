#ifndef BUZZBRIDGE_HPP
#define BUZZBRIDGE_HPP

#include "BuzzUtility.hpp"
#include <ros/ros.h>
#include <swarmus_ros_simulation/InterLocalization_grid.h>

#define BUZZRATE 10 // Frequency desired for buzz (in Hz)

struct BuzzFiles_t {
    std::string script;
    std::string byteCode;
    std::string debugCode;
};

class BuzzBridge {
protected:
    BuzzFiles_t m_BuzzFiles;
    void getROSParameters();

    void interlocGridCallback(const swarmus_ros_simulation::InterLocalization_grid &p_Grid);

    int m_RobotID;
    ros::NodeHandle* m_NodeHandle;
    ros::Subscriber m_InterlocalisationSubscriber;
public:
    BuzzBridge(ros::NodeHandle* p_NodeHandle);
    ~BuzzBridge();
    void execute(void);
    void registerHookFunctions();
};




#endif // BUZZBRIDGE_HPP