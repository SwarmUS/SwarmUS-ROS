cmake_minimum_required(VERSION 3.0.2)
project(swarmus_turtlebot)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        move_base
        move_base_msgs
        )

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
        )

target_include_directories(turtlebot_bridge
        PUBLIC include
        PRIVATE ${catkin_INCLUDE_DIRS}
        )
