cmake_minimum_required(VERSION 2.8.3)
project(wsg_50_interface)

# TODO: Should we need to do this?
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    controller_manager
    hardware_interface
    pr_ros_controllers
    pr_hardware_interfaces
    sensor_msgs
    wsg_50_driver
    )

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    std_msgs
    roscpp
    controller_manager
    hardware_interface
    sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${wsg_50_driver_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Declare a cpp executable
add_executable(weiss_hardware src/main.cpp src/WeissHand.cpp)

# Specify libraries to link a library or executable target against
 target_link_libraries(weiss_hardware
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${wsg_50_driver_LIBRARIES}
)
