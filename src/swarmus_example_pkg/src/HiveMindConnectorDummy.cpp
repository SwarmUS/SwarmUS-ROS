#include "ros/ros.h"
#include "hive_mind/ExampleMessage.h"
#include "swarmus_example_pkg/DummyUtils.h"

void hiveMindTopicCallback(const hive_mind::ExampleMessage& msg) {
    ROS_INFO("THE HIVEMIND TOLD ME :");
    ROS_INFO("%d", msg.number);
    ROS_INFO("%s", msg.text.c_str());

    // do some random stuff
    int incremented = DummyUtils::addTwoNumbers(2, msg.number);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "swarmus_example_pkg");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("exampleTopic", 1000, hiveMindTopicCallback);

    ros::spin();

    return 0;
}
