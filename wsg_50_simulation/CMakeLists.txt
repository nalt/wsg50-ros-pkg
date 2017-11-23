cmake_minimum_required(VERSION 2.8.3)
project(wsg_50_simulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  urdf
  wsg_50_common
)

catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES vh_pixelmap
  CATKIN_DEPENDS roscpp urdf std_msgs std_srvs wsg_50_common
#  DEPENDS system_lib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(wsg_50_sim_keyboard_teleop src/wsg_50_keyboard_teleop.cpp)
target_link_libraries(wsg_50_sim_keyboard_teleop ${catkin_LIBRARIES})
add_dependencies(wsg_50_sim_keyboard_teleop wsg_50_common_generate_messages_cpp)

add_executable(wsg_50_sim_driver src/wsg_50_sim_driver.cpp)
target_link_libraries(wsg_50_sim_driver ${catkin_LIBRARIES})
add_dependencies(wsg_50_sim_driver wsg_50_common_generate_messages_cpp)
