#include "hive_mind/ExampleMessage.h"
#include "ros/ros.h"
#include "swarmus_example_pkg/DummyUtils.h"

class HiveMindDummyConnector {
  public:
    HiveMindDummyConnector();

  private:
    ros::NodeHandle node_handle;
    ros::Publisher pub;
    ros::Subscriber sub;

    void hiveMindTopicCallback(const hive_mind::ExampleMessage& msg);
};

HiveMindDummyConnector::HiveMindDummyConnector() {
    pub = node_handle.advertise<hive_mind::ExampleMessage>("hiveMindConnectorDummy/exampleTopic",
                                                           1000);
    sub = node_handle.subscribe("hive_mind/exampleTopic", 1000,
                                &HiveMindDummyConnector::hiveMindTopicCallback, this);
}

void HiveMindDummyConnector::hiveMindTopicCallback(const hive_mind::ExampleMessage& msg) {
    ROS_INFO("The HiveMind told me :");
    ROS_INFO("%d", msg.number);
    ROS_INFO("%s", msg.text.c_str());

    // do some random stuff
    int incremented = DummyUtils::addTwoNumbers(2, msg.number);

    this->pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "swarmus_example_pkg");

    HiveMindDummyConnector connector;

    ros::spin();

    return 0;
}
