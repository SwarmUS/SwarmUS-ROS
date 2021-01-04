#include "swarmus_ros_simulation/interCommunication.h"

InterCommunication::InterCommunication() {
    ROS_INFO("HiveBoard communication initialization");

    robot_name = Simulation::GetParamRobotName();

    publisher = node_handle.advertise<swarmus_ros_simulation::Communication>("communication", 1000);
    subscriber = node_handle.subscribe("/CommunicationBroker/" + robot_name, 1000,
                                       &InterCommunication::communicationCallback, this);
}

void InterCommunication::publish(const swarmus_ros_simulation::Communication& msg)

{
    publisher.publish(msg);
}

void InterCommunication::communicationCallback(const std_msgs::String::ConstPtr& msg) {
    // DO SOMETHING AT MESSAGE RECEPTION
}

const std::string InterCommunication::getRobotName() { return robot_name; }

int main(int argc, char** argv) {
    ros::init(argc, argv, "hiveboard_interCommunications");

    InterCommunication interCommunication;
    std::string robot_name = interCommunication.getRobotName();

    // Send message each second
    ros::Rate loop_rate(1);

    // For current implementation, pioneer_0 only will send a message to pioneer_1.
    while (ros::ok()) {
        swarmus_ros_simulation::Communication msg;
        if (robot_name == "pioneer_0") {
            msg.source_robot = robot_name;
            msg.target_robot = Simulation::Communication::AllRobotsExceptSelf;
            msg.message = robot_name + " say hello.";

            // ROS_INFO("%s", msg.data.c_str());

            interCommunication.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
};