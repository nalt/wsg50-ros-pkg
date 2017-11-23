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

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_executable(wsg_50_ip src/main.cpp ${DRIVER_SOURCES})
target_link_libraries(wsg_50_ip ${catkin_LIBRARIES})
add_dependencies(wsg_50_ip wsg_50_common_generate_messages_cpp)

#add_executable(wsg_50_can src/main_can.cpp src/checksum.cpp src/msg.c src/common.cpp src/functions_can.cpp)
#add_executable(wsg_50_can src/main_can.cpp ${DRIVER_SOURCES_CAN})

#include_directories(/home/marc/peak-linux-driver-7.5/driver/src/)
#link_directories(/home/marc/peak-linux-driver-7.5/lib/)
#add_compile_flags(wsg_50_can -g -Wall)
#target_link_libraries(wsg_50_can pcan)
