#include "WeissHand.h"
#include <cmath>        // std::abs

using namespace std;


WeissHand::WeissHand(ros::NodeHandle nh)
: movehand_state(pr_hardware_interfaces::IDLE)
{
  // TODO: Call Patrick's core wrapper.
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
