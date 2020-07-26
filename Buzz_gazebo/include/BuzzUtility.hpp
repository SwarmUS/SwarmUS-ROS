/******************************************
 * Author: SwarmUS
 * Description: Contains functions  
 * used to control buzz execution through 
 * the ROS ecosystem. 
 ******************************************/

#ifndef BUZZ_UTILITY_HPP
#define BUZZ_UTILITY_HPP

#include <buzz/buzzdebug.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <stdint.h>
#include <string>
#include <ostream>
#include <ros/ros.h>

namespace BuzzUtility {

    int setBuzzScript(const char* bo_filename, const char* bdbg_filename, int robot_id);
    void buzzScriptStep();
    void buzzScriptDestroy();
    int buzzScriptDone();
    int getRobotID();
    int getSwarmSize();
    buzzvm_t getVM();
    std::string getVMState();
    const char* buzzErrorInfo();
    std::string compileBuzzScript(std::string p_bzzFilename);
    int registerHookFunction(const char* p_BuzzFunctionName, buzzvm_funp p_CallbackFunctionPointer);
    int buzzPrint(buzzvm_t vm);
    int addNeighbhor(uint16_t p_RobotId, float p_Distance_meters, float p_HorizontalAngle_radians, float p_VerticalAngle_radians = 0);


} // namespace buzz_utility


#endif // BUZZ_UTILITY_HPP