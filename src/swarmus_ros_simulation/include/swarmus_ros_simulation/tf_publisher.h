#include <map>
#include <string>

#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class SimulationTfPublisher
{
    public:
        void tfCallback(const gazebo_msgs::ModelStates& msg);

    private:
        std::map<std::string, bool> entity_publish_map_tf_list_;
        tf::TransformBroadcaster tfBroadcaster_;
        tf::TransformListener tfListener_;

        const bool getMapToWorldTransform(const std::string& entity_name, 
                                                                const tf::Transform& base_footprint_to_world_tf,
                                                                tf::Transform& map_to_world_tf);
        const tf::Transform getUnitTransform();

};