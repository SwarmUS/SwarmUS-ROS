cmake_minimum_required(VERSION 3.13)

# This is to make sure that catkin treats this as a ROS package. Sorry for the ugly hack.
find_package(catkin REQUIRED COMPONENTS
        std_msgs
        )

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
project(swarmus_pioneer)


