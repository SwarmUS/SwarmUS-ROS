# SwarmUS-ROS

SwarmUS-ROS contains all the ROS packages developed for the SwarmUS project. It also includes some Gazebo simulations.

# Requirements

* [Ubuntu 20.04 LTS](https://ubuntu.com/download). Development and setup for this repository was made in Ubuntu 20.04 "focal-fossa".
* [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu). Installing the package `ros-noetic-desktop-full` ensures that you have the Gazebo simulator, `rqt`, and `rviz` (required).
* [Python 3.x](https://www.python.org/downloads/)

## Repository structure

```
SwarmUS-ROS
├── contrib
│   └── HiveMind
│   └── mpu6050
│   └── realsense
│   └── roboclaw
│   └── rplidar
├── lgtm.yml
├── LICENSE
├── README.md
├── scripts
│    ├── install_dependencies.sh
├── src
│   ├── hiveboard_bridge
│   ├── swarmus_example_pkg
│   ├── swarmus_pioneer
│   ├── swarmus_ros_description
│   └── swarmus_ros_simulation
└── tools
    ├── check_format.py
    ├── config.py
    ├── contrib
    ├── format.py
    ├── generate_doc.py
```

This repo contains a `contrib` folder which refers to external code that needs to be included as source.

## ROS packages structure

The ROS packages follow the structure prescribed by the [ROS Wiki](http://wiki.ros.org/catkin/Tutorials/CreatingPackage):

```
swarmus_example_pkg
├── CMakeLists.txt
├── include
│   └── swarmus_example_pkg
│       └── DummyUtils.h
├── package.xml
├── src
│   ├── DummyUtils.cpp
│   └── HiveMindConnectorDummy.cpp
└── test
    └── DummyUtilsTests.cpp
```

### HiveMind
The [HiveMind repository](https://github.com/SwarmUS/HiveMind) is included as a git submodule, in the `contrib` folder. This repository contains a specific BSP that uses ROS nodes. This is to be used in a simulation environment. Thus, the simulation will use the "real" embedded code from the HiveMind.

## Building

Assuming you have a working ROS + Gazebo setup on you system, you need only clone this repository in your Catkin workspace, initialise the git submodules, and build it. You will also need to use [rosdep](http://wiki.ros.org/rosdep) to manage some external package dependencies. Refer to the following instruction for building:

```
cd ~/catkin_ws/src
git clone https://github.com/SwarmUS/SwarmUS-ROS.git
cd SwarmUS-ROS
git submodule update --init --recursive
sh scripts/install dependencies.sh
rosdep install --from-paths src --ignore-src -r -y
cd ~/catkin_ws && catkin_cmake
```
NOTE: the `install dependencies.sh` script will only works on Ubuntu-based system. It will ask for elevated permissions to install packages on some specific paths. It is recommended to read the script beforehand to make sure nothing harmful will be done to your system.
## Running unit tests

Unit tests are run via Catkin :

```
catkin_make run_tests
```

## Running simulations in Gazebo
As of now, there are some launch files located in the package `swarmus_ros_simulation` that spawn some robots in an empty scene in the Gazebo simulator. To launch the basic example, run the command:

```
roslaunch swarmus_ros_simulation multirobot_empty.launch
```

## Tools
This repository contains a `tools` folder with some python scripts that help accomplish a few tasks, mainly the generation of the documentation and some linting. Please refer to the scripts for some extended documentation.

## Documentation
An up to date version of the documentation can be found [here](https://swarmus.github.io/SwarmUS-ROS/index.html)

## Static analysis
The packages in this repository are analysed with [Haros](https://github.com/git-afsantos/haros/tree/master/haros). The results are published (here)[https://swarmus.github.io/SwarmUS-ROS-Haros-viz/#dashboard].