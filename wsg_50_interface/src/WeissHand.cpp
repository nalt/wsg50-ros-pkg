#include "WeissHand.h"
#include <cmath>        // std::abs

using namespace std;

/**
 * @brief read_hand_params: Reads hand parameters from parameter server
 * @param nh: Node handle used for accessing server
 * @param param_prefix: Prefix for parameter keys
 * @param hand_params: The data structure to store loaded values into
 * @return True if all required params were loaded, else false
**/
bool read_hand_params(ros::NodeHandle& nh, std::string param_prefix, Hand::hand_params_t& hand_params) {

    if(!nh.getParam(param_prefix+"/ip", hand_params.ip)) {
        ROS_ERROR("Did not get hand ip, aborting");
        return false;
    }
    if(!nh.getParam(param_prefix+"/port", hand_params.port)) {
        ROS_ERROR("Did not get hand port, aborting");
        return false;
    }
    if(!nh.getParam(param_prefix+"/local_port", hand_params.local_port)) {
        ROS_ERROR("Did not get hand local port, aborting");
        return false;
    }
    if(!nh.getParam(param_prefix+"/use_tcp", hand_params.use_tcp)) {
        ROS_ERROR("Did not get hand comm method, aborting");
        return false;
    }
    if(!nh.getParam(param_prefix+"/finger0_type", hand_params.finger0_type)) {
        ROS_ERROR("Did not get type of finger 0, aborting");
        return false;
    }
    if(!nh.getParam(param_prefix+"/finger1_type", hand_params.finger1_type)) {
        ROS_ERROR("Did not get type of finger 1, aborting");
        return false;
    }
    if(!nh.getParam(param_prefix+"/hand_data_buffer_size", hand_params.hand_data_buffer_size)) {
        ROS_ERROR("Did not get hand state buffer max size, aborting");
        return false;
    }
    if(!nh.getParam(param_prefix+"/hand_read_rate", hand_params.hand_read_rate)) {
        ROS_ERROR("Did not get hand read rate, aborting");
        return false;
    }

    return true;
}

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

  registerInterface(&jm_interface);

  // Start up Patrick's root API.
  Hand::hand_params_t hand_params;
  read_hand_params(nh, "hand0", hand_params);

  WeissFinger::weiss_finger_params_t* finger0_params = NULL;
	WeissFinger::weiss_finger_params_t* finger1_params = NULL;

  hand = std::make_shared<Hand>(&hand_params, finger0_params, finger1_params);
	hand->start_reading();
}

WeissHand::~WeissHand()
{
    ros::Duration(0.10).sleep();
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
  // TODO: What about sendFingerPositionCommand()???

  // NOTE: Speed is hard-coded for now.
  double speed = 30.0;
  hand->move_hand(command.at(0), speed);
}

void WeissHand::sendVelocityCommand(const std::vector<double>& command)
{
  // TODO.
}

void WeissHand::sendTorqueCommand(const std::vector<double>& command)
{
    // TODO.
}

void WeissHand::write(void)
{
    // TODO: Enable other modes.
    sendPositionCommand(cmd_pos);
}

void WeissHand::read(void)
{
    Hand::hand_data_t hand_status = hand->get_hand_state();

    // TODO: Seperate finger positions and other positions.
    pos[0] = hand_status.width;

    vel[0] = hand_status.speed;

    // TODO: This should probably be renamed.
    eff[0] = hand_status.force;
}
