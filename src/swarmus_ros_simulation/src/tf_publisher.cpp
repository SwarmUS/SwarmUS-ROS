#include "swarmus_ros_simulation/tf_publisher.h"

std::map<std::string, tf::Transform> tf_map;

tf::Transform getUnitTransform() {
    tf::Transform transform;

    transform.setRotation(tf::Quaternion::getIdentity());

    return transform;
}

void tfCallback(const gazebo_msgs::ModelStates& msg) {
    static tf::TransformBroadcaster s_tfBroadcaster;

    std::vector<std::string>::const_iterator it;

    for ( it = msg.name.begin(); it != msg.name.end(); ++it) {
        // Name of the child link: robot/odom
        std::string buf(it->c_str());
        buf.append("/odom");

        // Send transform relative to "world"
        s_tfBroadcaster.sendTransform(
            tf::StampedTransform(getUnitTransform(), ros::Time::now(), "world", buf));

        // Sleep for one nanosecond
        ros::Duration(0, 1).sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_publisher");

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/gazebo/model_states", 10, &tfCallback);

    ros::spin();
    return 0;
};