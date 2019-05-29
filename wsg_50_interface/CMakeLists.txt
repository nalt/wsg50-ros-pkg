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
# TODO: Jaco did this, do we need to?
# link_directories(${PROJECT_SOURCE_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE})

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

# TODO: Need something like this?
## Declare a cpp library
add_library(jaco_hardware
  src/WeissHand.cpp
)

# TODO: Need something like this?
## Declare a cpp executable
#add_executable(jaco_hardware src/main.cpp src/JacoRobot.cpp)

# TODO: Need something like this?
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(jaco_hardware_node jaco_hardware_generate_messages_cpp)
#set(KINOVA_ARM_LIBS ${catkin_LIBRARIES})
#set(KINOVA_ARM_LIBS
#    ${KINOVA_ARM_LIBS}
#    -l:USBCommLayerUbuntu.so
#    -l:USBCommandLayerUbuntu.so)

# TODO: Need something like this?
## Specify libraries to link a library or executable target against
# target_link_libraries(jaco_hardware
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${KINOVA_ARM_LIBS}
# )
