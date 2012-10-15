cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# WSG_50_TCP version

set(DRIVER_SOURCES 
  src/checksum.cpp
  src/cmd.c
  src/common.cpp
  src/functions.cpp
  src/interface.cpp
  src/main.cpp
  src/msg.c
  src/serial.c
  src/tcp.c
  src/udp.c)
  

rosbuild_add_executable(wsg_50_tcp src/main.cpp ${DRIVER_SOURCES})

# WSG_50_CAN version

set(DRIVER_SOURCES_CAN
  src/checksum.cpp
  src/common.cpp
  src/functions_can.cpp
  src/msg.c)

#rosbuild_add_executable(wsg_50_can src/main_can.cpp src/checksum.cpp src/msg.c src/common.cpp src/functions_can.cpp)
rosbuild_add_executable(wsg_50_can src/main_can.cpp ${DRIVER_SOURCES_CAN})

add_definitions(-DOSNAME_LINUX)

include_directories(/home/marc/peak-linux-driver-7.5/driver/src/)
link_directories(/home/marc/peak-linux-driver-7.5/lib/)

rosbuild_add_compile_flags(wsg_50_can -g -Wall)

target_link_libraries(wsg_50_can pcan)


