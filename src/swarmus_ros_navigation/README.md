
# swarmus_ros_navigation

## Overview

This package contains the nodes related to the navigation stack and the mapping nodes of our pioneer 2DX.

**Keywords:** Pioneer, Swarm, MoveBase, Navigation, move_base

### License
The source code is released under a [MIT License](SwarmUS-ROS/LICENSE).

**Author: SwarmUS<br />
Maintainer: SwarmUS, swarmus@usherbrooke.ca**

The swarmus_ros_navigation package has been tested under [ROS] melodic on Noetic on 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [move_base](http://wiki.ros.org/move_base) (navigation stack)
- [global_planner](http://wiki.ros.org/move_base) (Global trajectory planner)
- [actionlib](http://wiki.ros.org/move_base) (Interface between the desired goal and the navigation stack)
- [tf2](http://wiki.ros.org/tf2) (Transform libraries)
- [RTAB-Map](http://introlab.github.io/rtabmap/)

#### Building

To build from source, follow the building instructions in the [SwarmUS/SwarmUS-ROS](https://github.com/SwarmUS/SwarmUS-ROS) repo.


## Usage

To start the navigation stack (the move_base node) for the pioneer_0, run this command:

	roslaunch swarmus_ros_navigation navigation.launch

To start the navigation stack of a specific pioneer, run this command:

	roslaunch swarmus_ros_navigation navigation.launch robot_name:=pioneer_1

## Message files

* **MoveByMessage.msg**: Desired position of the robot in a x and y plane

## Launch files

* **navigation.launch:** Loads all the configuration files that will be used by the move_base node. This files contains parameters that is affecting on how the costmaps of move_base are build, how it plans global trajectories and how its local planner behaves.

     General arguments:

     - **`robot_name`**  Name of the robot that will be used to prefix the topics  and the needed frame. Default: `pioneer_0`.
     - **`use_map`**  Flag to indicate if the global costmap should used an occupancy grid topic and should receive goals in its map frame. Default: `false`.
     
     Parameters of the costmap are found inside the config directory:
     
     -  **`costmap_common_params.yaml`**  Config files that contains params used by the local and global costmap. Includes the params for all plugins used in the navigation stack.
     -  **`global_costmap_params.yaml`**  Config files that contains params used by global costmap. This config doesn't use a map as a layer so it will only plan depending on the obstacle layer and an inflation layer. Only loaded if **`use_map`** is false.
     -  **`global_costmap_with_map_params.yaml`** Config files that contains params used by global costmap. This config uses the map frame and an occupancy grid as a layer. It subscribes to the `grid_map` topic. Only loaded if **`use_map`** is true.
     -  **`local_costmap_params.yaml`**  Config files that contains params used by lobal costmap. Uses the obstacle layer and an inflation layer.
-  **`base_local_planer_params.yaml`**  Config files that contains params used by  `TrajectoryPlannerROS`.  Theses params dictate the behavior of the speed controller to follow its trajectory and avoid obstacles.
     -  **`base_global_planer_params.yaml`**  Config files that contains params used by  `BaseGlobalPlanner`.  Theses params indicates mainly which algorithm to use and how to interpret the global_costmap to create global trajectory.
     
     
     
* **rtabmap.launch:** Starts a RTAB-map to launch a slam session.

     General arguments:

     - **`robot_name`**  Name of the robot that will be used to prefix the topics  and the needed frame. Default: `pioneer_0`.

     Parameters of the costmap are defined in the [RTAB-Map wiki](http://wiki.ros.org/rtabmap_ros).

     

* **rviz_pioneer_0.launch**: Starts a rviz node to visualize the results of move_base costmaps, move_base plans and data of sensors in space.  

* **rviz_agent_1.launch**: Same as **rviz_pioneer_0.launch** but with the robot name changed to `agent_1`.  

* **rviz_pioneer_0_rtabmap.launch**: Starts a rviz node like **rviz_pioneer_0.launch** but the fixed frame is set to `robot_name/map`, move_base's goals are issued in this new frame and an occupancy grid is displayed.  
## Nodes

### Navigation

- Node that transform a x and y desired displacement of the robot and translate it in move_base goal.

#### Subscribed Topics

* **`/robot_name/navigation/moveBy`** (swarmus_ros_navigation::MoveByMessage)

	Displacement command in x and y command.


#### Published Topics

- **`/robot_name/move_base_simple/goal`** (geometry_msgs/PoseStamped)

  Goal given to the move_base_simple in the /base_footprint of the robot

#### Parameters

- **`~/robot_name`**(string, default:"pioneer_0")

  Name of the robot that owns the node

