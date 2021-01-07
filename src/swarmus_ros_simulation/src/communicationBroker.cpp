#include "swarmus_ros_simulation/communicationBroker.h"

CommunicationBroker::CommunicationBroker() {
    ROS_INFO("Communication Broker initialization");

    for (std::string& robot_name : Simulation::getRobotList()) {
        std::string publishing_topic = "CommunicationBroker/" + robot_name;
        std::string subscribing_topic = "/" + robot_name + "/communication";

        ros::Publisher pub = m_nodeHandle.advertise<std_msgs::String>(publishing_topic, 1000);
        ros::Subscriber sub = m_nodeHandle.subscribe(
            subscribing_topic, 1000, &CommunicationBroker::communicationCallback, this);

        m_publishersMap.emplace(robot_name, pub);
        m_subscribersList.push_back(sub);
    }
}

void CommunicationBroker::publishMsg(std::string robot_name,
                                     ros::Publisher pub,
                                     const swarmus_ros_simulation::Communication& msg) {
    std_msgs::String publishedMsg;
    publishedMsg.data = msg.message;
    pub.publish(publishedMsg);
}

void CommunicationBroker::communicationCallback(const swarmus_ros_simulation::Communication& msg) {
    if (msg.target_robot == Simulation::Communication::allRobots) {
        for (auto const& robotm_publisher : m_publishersMap)
            publishMsg(robotm_publisher.first, robotm_publisher.second, msg);
    } else if (msg.target_robot == Simulation::Communication::allRobotsExceptSelf) {
        for (auto const& robotm_publisher : m_publishersMap) {
            if (robotm_publisher.first != msg.source_robot)
                publishMsg(robotm_publisher.first, robotm_publisher.second, msg);
        }
    } else {
        auto pubIter = m_publishersMap.find(msg.target_robot);
        if (pubIter != m_publishersMap.end())
            publishMsg(msg.target_robot, pubIter->second, msg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "communication_broker");

    CommunicationBroker communicationBroker;
    ros::spin();

    return 0;
};