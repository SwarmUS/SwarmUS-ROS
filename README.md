# SwarmUS-ROS

SwarmUS-ROS contains all the ROS packages developed for the SwarmUS project. It also includes some Gazebo simulations.

# Requirements

* [Ubuntu 20.04 LTS](https://ubuntu.com/download). Development and setup for this repository was made in Ubuntu 20.04 "focal-fossa".
* [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu). Installing the package `ros-noetic-desktop-full` ensures that you have the Gazebo simulator, `rqt`, and `rviz` (required).
* [Python 3.x](https://www.python.org/downloads/)

## Building
Assuming you have a working ROS + Gazebo setup on you system, you need only clone this repository in your Catkin workspace and build it.

```
cd ~/catkin_ws/src
git clone https://github.com/SwarmUS/SwarmUS-ROS.git
cd ~/catkin_ws && catkin_cmake
```

## Running unit tests

Unit tests are run via Catkin :

```
catkin_make run_tests
```

## Tools
This repository contains a `tools` folder with some python scripts that help accomplish a few tasks, mainly the generation of the documentation and some linting. Please refer to the scripts for some extended documentation.

## Documentation
An up to date version of the documentation can be found [here](https://swarmus.github.io/SwarmUS-ROS/index.html)