#include "WeissHand.h"
#include <cmath>        // std::abs

using namespace std;


WeissHand::WeissHand(ros::NodeHandle nh)
: movehand_state(pr_hardware_interfaces::IDLE)
{
  ROS_INFO("Starting to initialize jaco_hardware");
  int i;
  cmd_pos.resize(num_hand_dof);
  cmd_vel.resize(num_hand_dof);
  cmd_eff.resize(num_hand_dof);
  zero_velocity_command.resize(num_hand_dof, 0.0);
  pos.resize(num_hand_dof);
  vel.resize(num_hand_dof);
  eff.resize(num_hand_dof);

  // connect and register the joint state interface.
  // this gives joint states (pos, vel, eff) back as an output.
  hardware_interface::JointStateHandle state_handle("wsg_50_joint_1", &pos[0], &vel[0], &eff[0]);
  jnt_state_interface.registerHandle(state_handle);
  registerInterface(&jnt_state_interface);

  // connect and register the joint position interface
  // this takes joint velocities in as a command.
  hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle("wsg_50_joint_1"), &cmd_vel[0]);
  jnt_vel_interface.registerHandle(vel_handle);
  registerInterface(&jnt_vel_interface);

  // connect and register the joint position interface
  // this takes joint positions in as a command.
  hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle("wsg_50_joint_1"), &cmd_pos[0]);
  jnt_pos_interface.registerHandle(pos_handle);
  registerInterface(&jnt_pos_interface);

  // TODO: Do we really need this?
  ROS_INFO("Register Effort Interface...");
  // TODO.

  // connect and register the joint position interface
  // this takes joint effort in as a command.
  hardware_interface::JointHandle eff_handle(jnt_state_interface.getHandle("wsg_50_joint_1"), &cmd_eff[0]);
  jnt_eff_interface.registerHandle(eff_handle);
  registerInterface(&jnt_eff_interface);

  // connect and register the joint mode interface
  // this is needed to determine if velocity or position control is needed.
  hardware_interface::JointModeHandle mode_handle("joint_mode", &joint_mode);
  jm_interface.registerHandle(mode_handle);

  // TODO: Why are the fingers seperate?
  pr_hardware_interfaces::PositionCommandHandle position_command_handle(
        "/wsg_50_joint_1", &movehand_state, &finger_pos);
  movehand_interface.registerHandle(position_command_handle);
  registerInterface(&movehand_interface);

  registerInterface(&jm_interface);

  // Start up Patrick's root API.
  // TODO!

}

ros::Time WeissHand::get_time(void)
{
    return ros::Time::now();
}

ros::Duration WeissHand::get_period(void)
{
    // TODO (sniyaz): What is a reasonable period?
    return ros::Duration(0.01);
}

void WeissHand::sendPositionCommand(const std::vector<double>& command)
{
  // TODO.
}

void WeissHand::sendFingerPositionCommand(const std::vector<double>& command)
{
  // TODO.
}

void WeissHand::sendVelocityCommand(const std::vector<double>& command)
{
  // TODO.
}

void WeissHand::sendTorqueCommand(const std::vector<double>& command)
{
    // TODO.
}
