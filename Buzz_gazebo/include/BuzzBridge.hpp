#ifndef BUZZBRIDGE_HPP
#define BUZZBRIDGE_HPP

#include "buzz_utility.hpp"
#include <ros/ros.h>

#define BUZZRATE 10 // Frequency desired for buzz (in Hz)

struct BuzzFiles_t {
    std::string script;
    std::string byteCode;
    std::string debugCode;
};

class BuzzBridge {
private:

protected:
    BuzzFiles_t m_BuzzFiles;
    void getROSParameters();

    int m_RobotID;
    ros::NodeHandle* m_NodeHandle;
public:
    BuzzBridge(ros::NodeHandle* p_NodeHandle);
    ~BuzzBridge();
    void execute(void);
    void registerHookFunctions();
};




#endif // BUZZBRIDGE_HPP