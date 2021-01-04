SwarmUS-ROS contains all the ROS packages developed for the SwarmUS project. It also includes some Gazebo simulations.

## Usage
Assuming you have a working ROS + Gazebo setup on you system, you need only clone this repository in your Catkin workspace and build it.

```
cd ~/catkin_ws/src
git clone https://github.com/SwarmUS/SwarmUS-ROS.git
cd ~/catkin_ws && catkin_cmake
```

## Tools
This repository contains a `tools` folder with some python scripts that help accomplish a few tasks, mainly the generation of the documentation and some linting. Please refer to the scripts for some extended documentation.