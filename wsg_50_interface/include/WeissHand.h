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
      shared_ptr<Hand> hand;

      hardware_interface::JointStateInterface jnt_state_interface;
      hardware_interface::EffortJointInterface jnt_eff_interface;
      hardware_interface::VelocityJointInterface jnt_vel_interface;
      hardware_interface::PositionJointInterface jnt_pos_interface;
      hardware_interface::JointModeInterface jm_interface;

      pr_hardware_interfaces::PositionCommandInterface movehand_interface;
      pr_hardware_interfaces::MoveState movehand_state;

      vector<double> cmd_pos;
      vector<double> cmd_vel;
      vector<double> cmd_eff;
      vector<double> pos; // contains full dof
      vector<double> finger_pos; // just fingers, used for finger position control
      vector<double> vel;
      vector<double> eff;
      vector<double> pos_offsets;
      vector<double> soft_limits;
      vector<double> zero_velocity_command;
      int joint_mode; // this tells whether we're in position or velocity control mode
      int last_mode;
};

#endif
