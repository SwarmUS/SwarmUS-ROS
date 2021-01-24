#include "swarmus_ros_simulation/tf_publisher.h"

std::map<std::string, tf::Transform> tf_map;

tf::Transform getUnitTransform() {
    tf::Transform transform;

    transform.setRotation(tf::Quaternion::getIdentity());

    return transform;
}

void tfCallback(const gazebo_msgs::ModelStates& msg) {
    static tf::TransformBroadcaster s_br;

    for (int i = 0; i < msg.name.size(); i++) {
        // Name of the child link: robot/odom
        std::string buf(msg.name[i].c_str());
        buf.append("/odom");

        // Send transform relative to "world"
        s_br.sendTransform(
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