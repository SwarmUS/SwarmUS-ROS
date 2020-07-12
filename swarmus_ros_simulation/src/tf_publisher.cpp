#include "swarmus_ros_simulation/tf_publisher.h"


std::map<std::string, tf::Transform> tf_map;

tf::Transform poseToTransform(geometry_msgs::Pose pose) {
    // TODO
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));

    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    transform.setRotation(q);

    return transform;
 /*
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
*/
}
 /*
void tfPublish() {
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}*/

void tfCallback(const gazebo_msgs::ModelStates& msg){
    // ROS_INFO("%s", msg.name[0].c_str());
    static tf::TransformBroadcaster br;

    for (int i = 0; i < msg.name.size(); i++) {
        std::string buf(msg.name[i].c_str());
        buf.append("/base_link");

        br.sendTransform(tf::StampedTransform(
            poseToTransform(msg.pose[i]), 
            ros::Time::now(), 
            "world", 
            buf));
        //std::map<char, int>::iterator it = m.find(msg.name[i]);
        /*
        if (it != tf_map.end()) { // la valeur existe
            it->second = poseToTransform(msg.name.pose[i]);
        } else { // miss
            tf_map.insert({
                msg.name[i].c_str(),
                poseToTransform(msg.name.pose[i])
            });
        }*/
        // TODO delete stale robots
    }
    
    // tfPublish();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "hiveboard_tfs");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/gazebo/model_states", 10, &tfCallback);

  ros::spin();
  return 0;
};