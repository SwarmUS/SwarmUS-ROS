
# swarmus_ros_navigation

## Overview

This package contains the nodes related to the navigation stack of our pioneer 2DX.

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

#### Building

To build from source, clone the latest version from the root of this repository into your catkin workspace and compile the package using

	cd catkin_ws/src
	git clone https://github.com/SwarmUS/SwarmUS-ROS.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make


## Usage

To start the navigation stack for the pioneer_0, run this command:

	roslaunch swarmus_ros_navigation navigation.launch

To start the navigation stack of a specific pioneer, run this command:

	roslaunch swarmus_ros_navigation navigation.launch robot_name:=pioneer_1

## Message files

* **MoveByMessage.msg**: Desired position of the robot in a x and y plane

## Launch files

* **navigation.launch:** Loads all the configuration files that will be used by the move_base node. This files contains parameters that is affecting on how the costmaps of move_base are build, how it plans global trajectories and how its local planner behaves.

     General arguments:

     - **`robot_name`**  Name of the first robot. Default: `pioneer_0`.
     - **`cmd_vel_topic`**  Name of the first robot. Default: `cmd_vel`.
     - **`odom_topic`**  Name of the first robot. Default: `odom`.
     - **`base_global_planner`** Type of global planner to use. Default: `global_planner/GlobalPlanner`.
     
     Parameters of the costmap (other parameters are found in the .config files):
     
     -  **`global_costmap/global_frame`**  Global reference frame in which the global costmap is build. Default: `robot_name/odom`.
     -  **`global_costmap/robot_base_frame`**  Frame used to represent the base of the robot in the global costmap. Default: `robot_name/base_footprint`.

     -  **`local_costmap/global_frame`**  Global reference frame in which the local costmap is build. Default: `robot_name/odom`.
     -  **`local_costmap/robot_base_frame`**  Frame used to represent the base of the robot in the local costmap. Default: `robot_name/base_footprint`.
     
     
     
* **rviz_pioneer_0.launch**: Starts a rviz node to visualize the results of move_base costmaps, move_base plans and data of sensors in space.  

     * The config of rviz can be found at swarmus_ros_navigation/rviz/pioneer_0_config.rviz
## Nodes

### navigation

- Node that transform a x and y desired deplacement of the robot and translate it in move_base goal.

#### Subscribed Topics

* **`/robot_name/navigation/moveBy`** (swarmus_ros_navigation::MoveByMessage)

	Deplacement command in x and y command.


#### Published Topics

- **`/robot_name/move_base_simple/goal`** (geometry_msgs/PoseStamped)

  Goal given to the move_base_simple in the /base_footprint of the robot

#### Parameters

- **`~/robot_name`**(string, default:"pioneer_0")

  Name of the robot that owns the node



## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/SwarmUS/SwarmUS-ROS/issues).

