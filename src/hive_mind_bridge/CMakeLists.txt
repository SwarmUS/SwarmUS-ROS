cmake_minimum_required(VERSION 3.13)
project(hive_mind_bridge)
set (CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  swarmus_ros_navigation
)

find_package(swarmus-hivemind-bridge CONFIG REQUIRED)
find_package(swarmus-propolis-cpp-common CONFIG REQUIRED)
find_package(swarmus-propolis-pheromones CONFIG REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# Main
add_executable(hive_mind_bridge src/main.cpp)
add_dependencies(hive_mind_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(hive_mind_bridge
        ${catkin_LIBRARIES}
        swarmus-propolis-pheromones
        swarmus-hivemind-bridge
        )

target_include_directories(hive_mind_bridge
        PUBLIC include
        PRIVATE ${catkin_INCLUDE_DIRS}
        )
