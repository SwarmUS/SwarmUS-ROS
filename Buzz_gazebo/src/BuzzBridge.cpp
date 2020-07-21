#include "BuzzBridge.hpp"

/*************************************************************************************************/
BuzzBridge::BuzzBridge(ros::NodeHandle* p_NodeHandle) {
    m_NodeHandle = p_NodeHandle;        // Not used for now but will be for topic subscriptions and publications
    getROSParameters();
    std::string filePath = BuzzUtility::compileBuzzScript(m_BuzzFiles.script);
    m_BuzzFiles.byteCode = filePath + ".bo";
    m_BuzzFiles.debugCode = filePath +".bdb";
    
}

/*************************************************************************************************/
BuzzBridge::~BuzzBridge() {
    BuzzUtility::buzzScriptDestroy();
}

/*************************************************************************************************/
void BuzzBridge::getROSParameters() {

    if(ros::param::get("~BzzFileName", m_BuzzFiles.script)) {
        ROS_INFO("Buzz script selected: %s", m_BuzzFiles.script.c_str());
    }
    else {
        ROS_ERROR("Provide a .bzz file to run in Launch file");
        ROS_INFO("Buzz script selected: %s", m_BuzzFiles.script.c_str());
        system("rosnode kill rosbuzz_node"); // Node name defined launch file
    }

    std::string name;
    if(ros::param::get("~name", name)) {
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
    if( BuzzUtility::setBuzzScript(m_BuzzFiles.byteCode.c_str(), m_BuzzFiles.debugCode.c_str(), m_RobotID) ) {
        registerHookFunctions();
        while(ros::ok() && !BuzzUtility::buzzScriptDone()){

            BuzzUtility::buzzScriptStep();

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


/*************************************************************************************************/
void BuzzBridge::registerHookFunctions(){
  // register more specified functions to be called from buzz
}