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
│   └── realsense-ros
│   └── roboclaw
│   └── rplidar
├── lgtm.yml
├── LICENSE
├── README.md
├── scripts
│    ├── install_dependencies.sh
│    ├── setup_librealsense.sh
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

  - If the Realsense packages can't be installed with the `apt-get install`command from the`install dependencies.sh` script , you might need to get the udev config directly from the [librealsense repo](https://github.com/IntelRealSense/librealsense). Run the following command from the Swarmus-ROS directory to clone the librealsense and set the udev rules: 

    ```
    sh scripts/setup_librealsense.sh
    ```

    If  you need the Realsense SDK follow the [librealsense source installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md). This might take some time since you will be building from the source code.

- `catkin_make_isolated` is used instead of `catkin_make` because there is a conflict when the Hivemind code and the hiveboard_bridge node are built since they both share the same code. You will need to source the devel_isolated setup whenever you build new packages with `catkin_make_isolated`.

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

## Running on Pioneer-2Dx

In the SwarmUS team, we use 2 modified Pioneer 2DX. One of the robot is equipped with a Realsense D455 and the other with a Realsense D435i. The package `swarmus_pioneer` and a part of the /contrib folder is used to launch the drivers to listen to sensors and to control the robot base. 

Run the following command and specify the camera being used (`d455`or `d435i`) to launch all the necessary drivers and nodes to control the robot: 

```
roslaunch swarmus_pioneer pioneer_bringup.launch camera_type:="d455"
```

Once launched, you can control and visualize the data of the sensors of the pioneer by running this command:

```
roslaunch swarmus_ros_navigation rviz_pioneer_0.launch
```

In Rviz, you can control directly the robot with the Teleop pannel in the bottom left corner or by using the *2D Nav Goal* command.



To launch a more completed launchfile that calls the robot bringup, a navigation stack, a [RTAB-Map]([rtabmap_ros - ROS Wiki](http://wiki.ros.org/rtabmap_ros)) node (SLAM) and all the necessary packages made by SwarmUS to connect the robot to the Swarm, execute the following command:

```
roslaunch swarmus_pioneer full_d455_swarm_robot_bringup.launch
```

or 

```
roslaunch swarmus_pioneer full_d435i_swarm_robot_bringup.launch
```

Once launched, you can control the robot, visualize its sensors' data, visualize the grid_map and the mapGraph made by RTAB-Map by running this command:

```
roslaunch swarmus_pioneer rviz_pioneer_0_rtabmap.launch
```

NOTE:

- The real robot has the `/pionner_0/` prefix on their nodes, tfs and topics to have better compatibility with the packages being developed in simulation.

#### Remote control

Sometimes, its useful to be connected to the ROS nodes being executed on the robot from a remote computer on the same network. To do so, the IP address of the robot and of the remote computer most be known. Run the following commands with the right addresses on the remote computer:

```
export ROS_IP=<remote_computer_ip_address>
export ROS_MASTER_URI=http://<robot_ip_address>:11311
```

Run the following command with the right address on the robot computer:

```
export ROS_IP=<robot_ip_address>
```

NOTE:

- The opened session will then communicate with the robot's ROS master whenever a ROS command is issue. So, running the `rostopic list` command will show topics being published on the robot. Furthermore, launching the Rviz's nodes shown above will execute and render Rviz on the remote computer, but it will display the robot's data.
- The environment variables set with the commands above will not be kept between sessions, so for each new terminal that needs to be connected to the robot's ROS master, you will need to rerun those commands.

## Tools

This repository contains a `tools` folder with some python scripts that help accomplish a few tasks, mainly the generation of the documentation and some linting. Please refer to the scripts for some extended documentation.

## Documentation

An up to date version of the documentation can be found [here](https://swarmus.github.io/SwarmUS-ROS/index.html)

## Static analysis

The packages in this repository are analysed with [Haros](https://github.com/git-afsantos/haros/tree/master/haros). The results are published (here)[https://swarmus.github.io/SwarmUS-ROS-Haros-viz/#dashboard].