#include "swarmus_ros_simulation/tf_publisher.h"


std::map<std::string, tf::Transform> tf_map;

tf::Transform poseToTransform(geometry_msgs::Pose pose) {
    // TODO
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));

    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    transform.setRotation(q);

    return transform;
}


void tfCallback(const gazebo_msgs::ModelStates& msg){
    static tf::TransformBroadcaster br;

    for (int i = 0; i < msg.name.size(); i++) {
        
        // Name of the child link: robot/base_footprint
        std::string buf(msg.name[i].c_str());
        buf.append("/base_footprint");

        // Send transform relative to "world"
        br.sendTransform(tf::StampedTransform(
            poseToTransform(msg.pose[i]), 
            ros::Time::now(), 
            "world", 
            buf));
        // TODO delete stale robots
    }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "hiveboard_tfs");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/gazebo/model_states", 10, &tfCallback);

  ros::spin();
  return 0;
};