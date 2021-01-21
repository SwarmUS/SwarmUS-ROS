
# swarmus_ros_simulation

## Overview

This package contains the nodes to simulate a swarm of Pioneer 2Dx equiped with our simulated Hiveboard. The purpose of the swarmus_ros_simulation package is to be able to simulate and test the swarm behavior from our Buzz script more rapidly and safely before implement it into real robots.

**Keywords:** Simulation, Pioneer, Gazebo, Swarm, Hiveboard, Interlocalization

### License
The source code is released under a [MIT License](SwarmUS-ROS/LICENSE).

**Author: SwarmUS<br />
Maintainer: SwarmUS, swarmus@usherbrooke.ca**

The swarmus_ros_simulation package has been tested under [ROS] melodic on Noetic on 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) (physics simulator)
- swarmus_ros_description
- swarmus_ros_navigation

#### Building

To build from source, clone the latest version from the root of this repository into your catkin workspace and compile the package using

	cd catkin_ws/src
	git clone https://github.com/SwarmUS/SwarmUS-ROS.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make



## Usage
To start the basic multirobot simulation, run

	roslaunch swarmus_ros_simulation multirobot_empty.launch

Then to visualize and control the pioneer_0, run

    roslaunch swarmus_ros_navigation rviz_pioneer_0.launch 

## Message files

* **Communication.msg**: Description of the message that the broker use for the simulated communication system

* **InterLocalization.msg**: Description of the message used for relative positionning

* **InterLocalization_grid.msg**:  Array of multiple InterLocalization.msg. This is used by the interlocalization system to indicate all relative position of a robot with all the other robots


## Launch files

* **multirobot_empty.launch:** Defines all the necessary argument to start a Gazebo in a empty world, calls an instance of Gazebo world, defines the parameters to instance multiple robot, calls a pioneer_spawner for each robot, starts the publishing of the TFs relative to the world frame and starts the communication broker to make robots talk to each other.

     Gazebo arguments

     - **`paused`**  Make the simulation start in a paused state. Default: `false`.
     
     -  **`use_sim_time`** Tells ROS nodes asking for time to get the Gazebo-published simulation time, published over the ROS topic /clock. Default: `true`.
     
     -  **`gui`**  Launch the user interface window of Gazebo. Default: `true`. *Note : The simulation can run without it and Rviz will still be able to visualize the world.*
     
     -  **`headless`** Enable gazebo state log recording. Default: `false`.
     
     -  **`debug`** Start gzserver (Gazebo Server) in debug mode using gdb. Default: `false`.

     Robot instance arguments and params
     -  **`model`** Tells ROS where to find the .URDF model of the robot Default: `$(find swarmus_ros_description)/urdf/pioneer.urdf.xacr`.
-  **`robot_0_name`**  Name of the first robot. Default: `pioneer_0`.
     - **`robot_1_name`**  Name of the first robot. Default: `pioneer_1`.
     **...**
     
     - **`robot_list`**  Array of all the robot names. This parameter is used by nodes that need of all active robots.

## Nodes

### tf_publisher

- Sets the transform between the /world frame and the /odom frame of all robots to 0 (same place, same orientation). This is done in order to have a common tf tree between all robots.

#### Subscribed Topics

* **`/gazebo/model_states`** (gazebo_msgs/ModelStates)

	Gives the pose of all robots. The callback attached to this topic is the one that publishes the transforms between the world and the odom/ frames.


#### Published Topics

- /**`tf`** (tf2_msgs/TFMessage)

  The transform between the world and /odom frames of robots.

### interLocalization

Measures the position of all the robots relatively to the owner of the node. Its purpose is to simulate what the Hiveboard will measure in real life.

#### Subscribed Topics

* **`/tf`** (tf2_msgs/TFMessage)

	Contains the transforms of all frames between robot.


#### Published Topics

- **`/robot_name/interlocalization_grid`** (swarmus_ros_simulation/InterLocalization_grid)

  List of the relative position and orientation between all robot and the owner of the node (a simulated robot)

- **`/robot_name/PolygonStamped`** (geometry_msgs/PolygonStamped)

  Geometric lines that start from the owner of the node to all robots. This is used in Rviz to visualize the relative position of other robots.

#### Parameters

* **`/robot_list`** (list of strings)

	List of the name of all robots.

- **`~/robot_name`**(string, default:"pioneer_0")

  Name of the robot that owns the node

### robot_name/interCommunication

Samll description of the node.

#### Subscribed Topics

* **`/subscribed topic`** ([link/toMsg])

	Add a small description


#### Published Topics

...



#### Parameters

* **`param1`** (string, default: "/param1_value")

	Description of the param

### communicationBroker

Samll description of the node.


#### Subscribed Topics

* **`/subscribed topic`** ([link/toMsg])

  Add a small description


#### Published Topics

...



#### Parameters

* **`param1`** (string, default: "/param1_value")

  Description of the param




## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/SwarmUS/SwarmUS-ROS/issues).

