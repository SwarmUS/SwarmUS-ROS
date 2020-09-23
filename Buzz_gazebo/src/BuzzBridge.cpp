#include "BuzzBridge.hpp"

#define DEGRESS_TO_RAD_FACTOR (0.01745F)

/*************************************************************************************************/
BuzzBridge::BuzzBridge(ros::NodeHandle* p_NodeHandle) {
    m_NodeHandle = p_NodeHandle;
    getROSParameters();
    std::string filePath = BuzzUtility::compileBuzzScript(m_RosParameters.bzzFileName.script);
    if(filePath == "error") {
        ROS_ERROR("Buzz compilation failed. Killing node");
        system("rosnode kill rosbuzz_node");
    }
    m_RosParameters.bzzFileName.byteCode = filePath + ".bo";
    m_RosParameters.bzzFileName.debugCode = filePath +".bdb";

    registerSubcriberCallbacks();

    m_MoveByPublisher = m_NodeHandle->advertise<swarmus_ros_navigation::MoveByMessage>(m_RosParameters.robot_name + "/navigation/moveBy", 1000);
}

/*************************************************************************************************/
BuzzBridge::~BuzzBridge() {
    BuzzUtility::buzzScriptDestroy();
}

/*************************************************************************************************/
void BuzzBridge::getROSParameters() {

    if(ros::param::get("~BzzFileName", m_RosParameters.bzzFileName.script)) {
        ROS_INFO("Buzz script selected: %s", m_RosParameters.bzzFileName.script.c_str());
    }
    else {
        ROS_ERROR("Provide a .bzz file to run in Launch file");
        ROS_INFO("Buzz script selected: %s", m_RosParameters.bzzFileName.script.c_str());
        system("rosnode kill rosbuzz_node"); // Node name defined launch file
    }

    if(ros::param::get("~robot_name", m_RosParameters.robot_name)) {
        ROS_INFO("Robot name provided: %s", m_RosParameters.robot_name.c_str());
        sscanf(m_RosParameters.robot_name.c_str(), "pioneer_%d", &m_RosParameters.robotID); // extract number after name
    }
    else {
        system("rosnode kill rosbuzz_node"); // Node name defined launch file
    }

    // Add cases for other possible parameters as needed
}


/*************************************************************************************************/
void  BuzzBridge::registerSubcriberCallbacks() {
    std::string topic =  "/" + m_RosParameters.robot_name + "/interlocalization_grid";
    ROS_INFO("Subscribing to: %s", topic.c_str());
    m_InterlocalisationSubscriber = m_NodeHandle->subscribe(topic.c_str(), 1000, &BuzzBridge::interlocGridCallback, this);

    topic = "/CommunicationBroker/" + m_RosParameters.robot_name;
    ROS_INFO("Subscribing to: %s", topic.c_str());
    //m_IntercommunicationSubscriber = m_NodeHandle->subscribe(topic.c_str(), 1000, &BuzzBridge::interCommunicationCallback, this);
}

/*************************************************************************************************/
void BuzzBridge::execute() {
    ros::Rate loopRate(BUZZRATE);
    if( BuzzUtility::setBuzzScript(m_RosParameters.bzzFileName.byteCode.c_str(), m_RosParameters.bzzFileName.debugCode.c_str(), m_RosParameters.robotID) ) {
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
  //BuzzUtility::registerHookFunction("moveBy", moveByClosure);
}

/*************************************************************************************************/
void BuzzBridge::interlocGridCallback(const swarmus_ros_simulation::InterLocalization_grid &p_Grid){
    for(swarmus_ros_simulation::InterLocalization msg : p_Grid.otherRobots) {
        BuzzUtility::addNeighbhor(stoi(msg.target_robot.erase(0,8)), msg.distance, msg.angle * DEGRESS_TO_RAD_FACTOR); 
    }
}

/*************************************************************************************************/
void BuzzBridge::interCommunicationCallback(const std_msgs::String::ConstPtr& msg) {   
    //ROS_INFO("Message receveived: %s", msg->data.c_str());
    //TODO: handle received message
}

/*************************************************************************************************/
int BuzzBridge::moveByClosure(buzzvm_t vm){
    swarmus_ros_navigation::MoveByMessage msg;
    
    buzzvm_lload(vm, 1);
    msg.distance_x = buzzvm_stack_at(vm, 1)->f.value;
    buzzvm_pop(vm);

    buzzvm_lload(vm, 2);
    msg.distance_y = buzzvm_stack_at(vm, 1)->f.value;
    buzzvm_pop(vm);

    m_MoveByPublisher.publish(msg);

    return buzzvm_ret0(vm);
}