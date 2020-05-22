cmake_minimum_required(VERSION 2.8.3)
project(wsg_50_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  wsg_50_common
  actionlib
)

catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES vh_pixelmap
  CATKIN_DEPENDS roscpp std_msgs std_srvs wsg_50_common
#  DEPENDS system_lib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


# WSG_50_TCP version
set(DRIVER_SOURCES
  src/checksum.cpp include/wsg_50/checksum.h
  src/cmd.c include/wsg_50/cmd.h
  src/common.cpp include/wsg_50/common.h
  src/functions.cpp include/wsg_50/functions.h
  src/interface.cpp include/wsg_50/interface.h
  src/gripper_communication.cpp include/wsg_50/gripper_communication.h
  src/gripper_action_server.cpp include/wsg_50/gripper_action_server.h
  src/gripper_standard_action_server.cpp include/wsg_50/gripper_standard_action_server.h
  src/gripper_socket.cpp include/wsg_50/gripper_socket.h
  include/wsg_50/node_state.h
  src/main.cpp
  src/msg.c include/wsg_50/msg.h
  src/serial.c include/wsg_50/serial.h
  src/tcp.c include/wsg_50/tcp.h
  src/udp.c include/wsg_50/udp.h)

# WSG_50_CAN version
set(DRIVER_SOURCES_CAN
  src/checksum.cpp
  src/common.cpp
  src/functions_can.cpp
  src/msg.c)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DOSNAME_LINUX)
add_definitions(-g)

# C++11 Needed
include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)

if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

add_executable(wsg_50_ip src/main.cpp ${DRIVER_SOURCES})
target_link_libraries(wsg_50_ip ${catkin_LIBRARIES})
add_dependencies(wsg_50_ip wsg_50_common_generate_messages_cpp)
install(TARGETS
  wsg_50_ip
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES
  launch/grippercontroller.yaml
  launch/wsg_50_tcp.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  nodes/wsg50_acknowledge_gripper_error_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#add_executable(wsg_50_can src/main_can.cpp src/checksum.cpp src/msg.c src/common.cpp src/functions_can.cpp)
#add_executable(wsg_50_can src/main_can.cpp ${DRIVER_SOURCES_CAN})

#include_directories(/home/marc/peak-linux-driver-7.5/driver/src/)
#link_directories(/home/marc/peak-linux-driver-7.5/lib/)
#add_compile_flags(wsg_50_can -g -Wall)
#target_link_libraries(wsg_50_can pcan)
