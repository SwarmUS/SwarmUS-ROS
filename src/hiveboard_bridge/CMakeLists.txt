cmake_minimum_required(VERSION 3.0.2)
project(hiveboard_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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

set(HIVEBOARD_BRIDGE_SOURCES
        src/main.cpp
        src/TCPServer.cpp
        src/TCPServerMonitor.cpp)

add_executable(hiveboard_bridge ${HIVEBOARD_BRIDGE_SOURCES})
add_dependencies(hiveboard_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(hiveboard_bridge ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hiveboard_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
