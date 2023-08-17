#include <map>
#include <string>

#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class SimulationTfPublisher
{
    public:
        /**
         * @brief Callback of the Gazebo message to publish transform between the world tf and all models broadcasted by Gazebo.
         * @param msg The message ModelStates of Gazebo
         */
        void tfCallback(const gazebo_msgs::ModelStates& msg);

    private:
        std::map<std::string, bool> entity_publish_map_tf_list_;
        tf::TransformBroadcaster tfBroadcaster_;
        tf::TransformListener tfListener_;

        /**
         * @brief Compute the transform between the map tranform of a Gazebo model and the world model from the Gazebo message. 
         * @param entity_name Name of the model in Gazebo and the tf frames. Used to listen to the tf frames.
         * @param base_footprint_to_world_tf Transform of the absolute and correct position between the model and the world.
         * @param map_to_world_tf Returned value of the transform between the map of the entity and the world.
         * @retval false Error, could not lookup the transform of the entity.
         * @retval true OK
         */
        const bool getMapToWorldTransform(const std::string& entity_name, 
                                                                const tf::Transform& base_footprint_to_world_tf,
                                                                tf::Transform& map_to_world_tf);
        const tf::Transform getUnitTransform();

};