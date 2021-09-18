cmake_minimum_required(VERSION 3.0.2)
project(swarmus_turtlebot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtlebot_bridge src/main.cpp)
add_dependencies(turtlebot_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlebot_bridge
        ${catkin_LIBRARIES}
        )

target_include_directories(turtlebot_bridge
        PUBLIC include
        PRIVATE ${catkin_INCLUDE_DIRS}
        )
