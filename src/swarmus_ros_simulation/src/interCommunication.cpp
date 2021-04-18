#include "swarmus_ros_simulation/interCommunication.h"

InterCommunication::InterCommunication() {
    ROS_INFO("HiveMind communication initialization");

    m_robotName = Simulation::getParamRobotName();

    m_publisher =
        m_nodeHandle.advertise<swarmus_ros_simulation::Communication>("communication", 1000);
    m_subscriber = m_nodeHandle.subscribe("/CommunicationBroker/" + m_robotName, 1000,
                                          &InterCommunication::communicationCallback, this);
}

void InterCommunication::publish(const swarmus_ros_simulation::Communication& msg) {
    m_publisher.publish(msg);
}

void InterCommunication::communicationCallback(const std_msgs::String::ConstPtr& msg) {
    // DO SOMETHING AT MESSAGE RECEPTION
}

std::string InterCommunication::getRobotName() { return m_robotName; }

int main(int argc, char** argv) {
    ros::init(argc, argv, "hiveboard_interCommunications");

    InterCommunication interCommunication;
    std::string m_robotName = interCommunication.getRobotName();

    // Send message each second
    ros::Rate loop_rate(1);

    // For current implementation, pioneer_1 only will send a message to pioneer_1.
    while (ros::ok()) {
        swarmus_ros_simulation::Communication msg;
        if (m_robotName == "pioneer_1") {
            msg.source_robot = m_robotName;
            msg.target_robot = Simulation::Communication::allRobotsExceptSelf;
            msg.message = m_robotName + " say hello.";

            // ROS_INFO("%s", msg.data.c_str());

            interCommunication.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
};