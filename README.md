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
│   └── realsense
│   └── roboclaw
│   └── rplidar
├── lgtm.yml
├── LICENSE
├── README.md
├── scripts
│    ├── install_dependencies.sh
│    ├── set_udev_rules.sh
│    └── 40-swarmus_pioneer.rules
├── src
│   ├── hiveboard_bridge
│   ├── swarmus_example_pkg
│   ├── swarmus_pioneer
│   ├── swarmus_ros_description
|   ├── swarmus_ros_navigation
│   └── swarmus_ros_simulation
└── tools
    ├── check_format.py
    ├── config.py
    ├── contrib
    ├── format.py
    └── generate_doc.py
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
sh scripts/install_dependencies.sh
rosdep install --from-paths src --ignore-src -r -y
cd ~/catkin_ws && catkin_make_isolated
echo "source ~/catkin_ws/devel_isolated/setup.bash" >> ~/.bashrc
source ~/.bashrc
sh scripts/set_udev_rules.sh
```
NOTE: 

- The `install dependencies.sh` and `set_udev_rules.sh` scripts will only work on Ubuntu-based system. It will ask for elevated permissions to install packages on some specific paths. It is recommended to read the script beforehand to make sure nothing harmful will be done to your system.

  - If the Realsense packages can't be installed with the `apt-get install`command, follow the [librealsense source installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md). This might take some time since you will be building from the source code.

- `catkin_make_isolated` is used instead of `catkin_make` because there is a conflict when the Hivemind code and the hiveboard_bridge node are built since they both share the same code. You will need to source the devel_isolated setup whenever you build new packages with `catkin_make_isolated`. You can add the command to the .bashrc for more convenience : 

  ```
  echo "source ~/catkin_ws/devel_isolated/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

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

When the simulation is launched, you can control and visualize the data of the sensors of the pioneer_0 by running this command:

```
roslaunch swarmus_ros_navigation rviz_pioneer_0.launch
```

In Rviz, you can control directly the robot with the Teleop pannel in the bottom left corner or by using the *2D Nav Goal* command.

## Tools

This repository contains a `tools` folder with some python scripts that help accomplish a few tasks, mainly the generation of the documentation and some linting. Please refer to the scripts for some extended documentation.

## Documentation
An up to date version of the documentation can be found [here](https://swarmus.github.io/SwarmUS-ROS/index.html)

## Static analysis
The packages in this repository are analysed with [Haros](https://github.com/git-afsantos/haros/tree/master/haros). The results are published (here)[https://swarmus.github.io/SwarmUS-ROS-Haros-viz/#dashboard].
