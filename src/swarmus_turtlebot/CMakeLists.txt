cmake_minimum_required(VERSION 3.13)
project(swarmus_turtlebot)
set (CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        move_base
        move_base_msgs
        )

find_package(swarmus-hivemind-bridge CONFIG REQUIRED)
find_package(swarmus-propolis-cpp-common CONFIG REQUIRED)
find_package(swarmus-propolis-pheromones CONFIG REQUIRED)

add_message_files(
        FILES
        MoveByMessage.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        move_base_msgs
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
)

include_directories(
        include ${swarmus_turtlebot_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_executable(turtlebot_bridge
        src/main.cpp
        src/Navigation.cpp)

add_dependencies(turtlebot_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlebot_bridge
        ${catkin_LIBRARIES}
        swarmus-propolis-pheromones
        swarmus-hivemind-bridge
        )

target_include_directories(turtlebot_bridge
        PUBLIC include
        PRIVATE ${catkin_INCLUDE_DIRS}
        )
