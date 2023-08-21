
# swarmus_ros_simulation

## Overview

This package contains the nodes to simulate a swarm of Pioneer 2Dx equiped with our simulated Hiveboard. The purpose of the swarmus_ros_simulation package is to be able to simulate and test the swarm behavior from our Buzz script more rapidly and safely before implementing it into real robots.

## Dependencies 

The swarmus_ros_simulation package has been tested under [ROS] melodic on 18.04 and Noetic on 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


Before usage, you should install the required dependencies for SwarmUS-ROS, check how to use the script to bootstrap and install the dependencies in `SwarmUS-ROS/README.md`

## Usage

### Simple swarm
To start a simple multirobot swarm simulation, run:
```sh
roslaunch swarmus_ros_simulation simple_swarm.launch
```

This will create a gazebo world with 4 agents. Each agents is a Pioneer 2Dx connected to a HiveMind and uses HiveConnect to communicate with the rest of the swarm. If you want a particular swarm behavior, you will need to change the Buzz code in `SwarmUS-ROS/contrib/HiveMind/src/bittybuzz/buzz_scripts/main.bzz`

### Swarm teleoperation from phone
To control the swarm of 4 agents from an android phone (who is the 5th agent) with HiveAR you can use:

``` sh
roslaunch swarmus_ros_simulation teleop_from_phone_multi_robot.launch 
```

Note that you will probably need to changed the hivemind_host_address to the IP address shown in HiveAR. Also make sure the computer you are simulating on and your android device are on the same network.


You can always use rviz, rqt, rostopic, rosparam or any other ROS tools to get more information on the running nodes, parameters, publisher and listener.

