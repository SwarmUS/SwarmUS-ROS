cmake_minimum_required(VERSION 3.0.2)
project(swarmus_example_pkg)

find_package(catkin REQUIRED COMPONENTS
  hive_mind
  roscpp
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES swarmus_example_pkg
#  CATKIN_DEPENDS hive_mind roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

set(HIVE_MIND_CONNECTOR_DUMMY_SOURCES
        src/DummyUtils.cpp
        src/HiveMindConnectorDummy.cpp)

add_executable(hive_mind_connector_dummy ${HIVE_MIND_CONNECTOR_DUMMY_SOURCES})

add_dependencies(hive_mind_connector_dummy ${catkin_EXPORTED_TARGETS})

target_link_libraries(hive_mind_connector_dummy ${catkin_LIBRARIES})

#############
## Testing ##
#############

catkin_add_gtest(DummyUtilsTests
        test/DummyUtilsTests.cpp
        src/DummyUtils.cpp)

if(TARGET DummyUtilsTests)
    target_link_libraries(DummyUtilsTests 
    ${catkin_LIBRARIES} 
    gtest_main)
endif()
