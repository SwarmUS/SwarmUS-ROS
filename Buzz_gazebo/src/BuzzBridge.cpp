#include "BuzzBridge.hpp"

/*************************************************************************************************/
BuzzBridge::BuzzBridge(ros::NodeHandle* p_NodeHandle) {
    m_NodeHandle = p_NodeHandle;
    getROSParameters();
    std::string filePath = buzz_utility::compileBuzzScript(m_BuzzFiles.script);
    m_BuzzFiles.byteCode = filePath + ".bo";
    m_BuzzFiles.debugCode = filePath +".bdb";
}

/*************************************************************************************************/
BuzzBridge::~BuzzBridge() {
    buzz_utility::buzzScriptDestroy();
}

/*************************************************************************************************/
void BuzzBridge::getROSParameters() {

    if(m_NodeHandle->getParam("BzzFileName", m_BuzzFiles.script)) {
        // Buzz script provided
    }
    else {
        ROS_ERROR("Provide a .bzz file to run in Launch file");
        system("rosnode kill rosbuzz_node"); // Node name defined launch file
    }

    std::string name;
    if(m_NodeHandle->getParam("name", name)) {
        m_RobotID = stoi(name.erase(0, 5)); // extract number after robot (name:robot1 => m_RobotID:1)
    }
    else {
        ROS_ERROR("Provide a .bzz file to run in Launch file");
        system("rosnode kill rosbuzz_node"); // Node name defined launch file
    }

    // Add cases for other possible parameters as needed
}

/*************************************************************************************************/
void BuzzBridge::execute() {
    ros::Rate loopRate(BUZZRATE);
    if( buzz_utility::setBuzzScript(m_BuzzFiles.byteCode.c_str(), m_BuzzFiles.debugCode.c_str(), m_RobotID) ) {
        while(ros::ok() && !buzz_utility::buzzScriptDone()){

            buzz_utility::buzzScriptStep();

            // Call functions to publish topics

            ros::spinOnce(); // Execute all callbacks in callback queue

            // Manage loopRate
            loopRate.sleep();
            if (loopRate.cycleTime() > ros::Duration(1.0 / (float)BUZZRATE)){
                ROS_WARN("ROSBuzz loop could not reach its desired rate of %dHz", BUZZRATE);
            }   
        }
    }
}