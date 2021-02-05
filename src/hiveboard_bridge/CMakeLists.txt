cmake_minimum_required(VERSION 3.0.2)
project(hiveboard_bridge)
set (CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include(cmake/propolis.cmake)
propolis_fetch_populate()

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
        src/StreamListener.cpp
        src/MessageHandler.cpp
        src/ReceiveThreadAction.cpp)

add_executable(hiveboard_bridge ${HIVEBOARD_BRIDGE_SOURCES})
add_dependencies(hiveboard_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(hiveboard_bridge
        ${catkin_LIBRARIES}
        SwarmUS::Propolis::Pheromones::HiveMind::Host
        )

target_include_directories(hiveboard_bridge
        PUBLIC include
        PRIVATE ${catkin_INCLUDE_DIRS}
        )

#############
## Testing ##
#############

# MessageHandlerUnitTest
add_executable(MessageHandlerUnitTest
        test/MessageHandlerUnitTest.cpp
        src/MessageHandler.cpp)

add_test(NAME MessageHandlerUnitTest COMMAND MessageHandlerUnitTest)

target_link_libraries(MessageHandlerUnitTest
        ${catkin_LIBRARIES}
        SwarmUS::Propolis::Pheromones::HiveMind::Host
        gmock_main
        )

# StreamListenerUnitTest
add_executable(ReceiveThreadActionUnitTest
        test/ReceiveThreadActionUnitTest.cpp
        src/ReceiveThreadAction.cpp)

add_test(NAME ReceiveThreadActionUnitTest COMMAND ReceiveThreadActionUnitTest)

target_link_libraries(ReceiveThreadActionUnitTest
        ${catkin_LIBRARIES}
        SwarmUS::Propolis::Pheromones::HiveMind::Host
        gmock_main
        )