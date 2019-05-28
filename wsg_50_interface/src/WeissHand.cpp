#include "JacoRobot.h"
#include <cmath>        // std::abs

using namespace std;


JacoRobot::JacoRobot(ros::NodeHandle nh)
: movehand_state(pr_hardware_interfaces::IDLE)
{
  // TODO: Call Patrick's core wrapper.
}

ros::Time JacoRobot::get_time(void)
{
    return ros::Time::now();
}

ros::Duration JacoRobot::get_period(void)
{
    // TODO (sniyaz): What is a reasonable period?
    return ros::Duration(0.01);
}

void JacoRobot::sendPositionCommand(const std::vector<double>& command)
{
  // TODO.
}

void JacoRobot::sendFingerPositionCommand(const std::vector<double>& command)
{
  // TODO.
}

void JacoRobot::sendVelocityCommand(const std::vector<double>& command)
{
  // TODO.
}

void JacoRobot::sendTorqueCommand(const std::vector<double>& command)
{
    // TODO.
}
