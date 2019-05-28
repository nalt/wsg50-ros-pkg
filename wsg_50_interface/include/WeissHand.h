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

// Low-level weiss API.
#include "wsg_50/hand.h"

// c++
#include <stdexcept>
#include <limits>
#include <iostream>

using namespace std;

class WeissHand: public hardware_interface::RobotHW
{
public:
      WeissHand(ros::NodeHandle nh);

      virtual ~WeissHand();

      ros::Time get_time(void);

      ros::Duration get_period(void);

      void sendPositionCommand(const std::vector<double>& command);
      void sendVelocityCommand(const std::vector<double>& command);
      void sendTorqueCommand(const std::vector<double>& command);
      void sendFingerPositionCommand(const std::vector<double>& command);

      void write(void);
      void read(void);

private:
      Hand hand;
};

#endif
