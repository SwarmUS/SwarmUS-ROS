#include <swarmus_ros_simulation/tf_publisher.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>


const tf::Transform SimulationTfPublisher::getUnitTransform()
{
    tf::Transform transform;

    transform.setRotation(tf::Quaternion::getIdentity());

    return transform;
}

const bool SimulationTfPublisher::getMapToWorldTransform(const std::string& entity_name, 
                                                                const tf::Transform& base_footprint_to_world_tf,
                                                                tf::Transform& map_to_world_tf)
{
    tf::StampedTransform map_to_base_footprint_stamped_tf; 
    try {
        tfListener_.lookupTransform(entity_name + "/base_footprint", entity_name+"/map", ros::Time(0), map_to_base_footprint_stamped_tf);
        map_to_world_tf = base_footprint_to_world_tf*map_to_base_footprint_stamped_tf;
        return true;
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    return false;
}

void SimulationTfPublisher::tfCallback(const gazebo_msgs::ModelStates& msg)
{
    std::vector<std::string>::const_iterator it_name;
    std::vector<geometry_msgs::Pose>::const_iterator it_pose = msg.pose.begin();

    for (it_name = msg.name.begin(); it_name != msg.name.end(); ++it_name) {
        const geometry_msgs::Pose entity_pose = *it_pose; 
        const std::string entity_name(*it_name);
        bool has_map_tf = false;

        if (entity_publish_map_tf_list_.find(entity_name) != entity_publish_map_tf_list_.end()) 
        {
            // Check if we registered that this entity has a map frame
            has_map_tf = entity_publish_map_tf_list_[entity_name];
        }
        else
        {
            // Register if the node has a map transform
            ros::param::get(entity_name+"/use_map_tf", has_map_tf);
            entity_publish_map_tf_list_[entity_name] = has_map_tf;
            if (has_map_tf)
            {
                ROS_INFO_STREAM("Sim_tf_pub: Map frame is expected for " << entity_name);
            }
            else
            {
                ROS_INFO_STREAM("Sim_tf_pub: Map frame is not expected for "<< entity_name);
            }
        }

        if (has_map_tf)
        {
            /* Change the transform between the map and the world so the transform between 
             the base_link and the world is consistent with Gazebo*/
            
            tf::Transform entity_to_world_tf;
            tf::Transform map_to_world_tf;
            tf::poseMsgToTF(entity_pose, entity_to_world_tf);

            if(getMapToWorldTransform(entity_name, entity_to_world_tf, map_to_world_tf))
            {
                tfBroadcaster_.sendTransform(
                    tf::StampedTransform(map_to_world_tf, ros::Time::now(), "world", entity_name+"/map"));
            }
        }
        else
        {
            // Send transform relative to "world"
            tfBroadcaster_.sendTransform(
                tf::StampedTransform(getUnitTransform(), ros::Time::now(), "world", entity_name+"/odom"));
        }

        // Sleep for one nanosecond
        ros::Duration(0, 1).sleep();
        
        // Point towards next pose
        ++it_pose;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_publisher");

    SimulationTfPublisher simTfPublisher;
    ROS_INFO_STREAM("Simulation TF publisher initialization");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/gazebo/model_states", 1, &SimulationTfPublisher::tfCallback, &simTfPublisher);

    ros::spin();
    return 0;
};