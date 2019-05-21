#ifndef WEISS_HAND_H
#define WEISS_HAND_H

// ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <pr_ros_controllers/joint_mode_interface.h>
#include <pr_hardware_interfaces/PositionCommandInterface.h>

//#include <hardware_interface/controller_info_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

// ros
#include <ros/ros.h>
#include <ros/console.h>

// TODO: Low-level weiss API.

// c++
#include <stdexcept>
#include <limits>
#include <iostream>

using namespace std;

class WeissHand: public hardware_interface::RobotHW
{
  // TODO
};

#endif
