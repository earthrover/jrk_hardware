
#include <algorithm>
#include <exception>
#include <sstream>
#include <stdexcept>

#include "ros/ros.h"

#include "jrk_hardware/jrk_hardware.h"

namespace jrk
{

JrkHardware::JrkHardware(std::map<std::string, std::string> joint_params, double conversion_factor)
  : joints()
  , conversion_factor(conversion_factor)
{
  if (joint_params.size() < 1)
  {
    throw std::invalid_argument("JrkHardware requires at least 1 joint.");
  }

  for (const auto& params : joint_params)
  {
    joints.emplace_back(new Joint(params.first, params.second));
  }
  registerInterfaces();

  active = true;
}

JrkHardware::~JrkHardware()
{
  stop();
}

void JrkHardware::registerInterfaces()
{
  for (auto& j : joints)
  {
    hardware_interface::JointStateHandle joint_state_handle(j->name, &j->pos, &j->vel, &j->eff);
    joint_state_interface.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle, &j->cmd);
    velocity_joint_interface.registerHandle(joint_handle);
  }

  registerInterface(&joint_state_interface);
  registerInterface(&velocity_joint_interface);
}

void JrkHardware::stop()
{
  if (!active) return;
  active = false;
  for (auto& j : joints)
  {
    try {
      j->jrk.motorOff();
    } catch (const jrk::JrkTimeout&) {
      handle_timeout(j->name, "sending stop");
    }
  }
  ROS_WARN_STREAM("sending commands to motors has been disabled");
}

void JrkHardware::handle_timeout(const std::string joint_name, std::string details)
{
  ROS_ERROR_STREAM("joint " << joint_name << " timed out " << details);
  stop();
}

void JrkHardware::clear_errors()
{
  for(auto& j : joints)
  {
    try {
      j->jrk.setTarget(2048);
    } catch (const jrk::JrkTimeout&) {
      handle_timeout(j->name, "clearing errors");
    }
    // there should be no errors now, otherwise exit
    if ((j->error = j->jrk.getErrorsHalting()))
    {
      ROS_WARN_STREAM("unable to clear errors on joint " << j->name);
      ROS_WARN_STREAM("sending commands to motors remains disabled");
      return;
    }
    else
    {
      ROS_INFO_STREAM("errors cleared on joint " << j->name);
    }
  }
  ROS_INFO("sending commands to motors has been re-enabled");
  active = true;
}

void JrkHardware::raw_feedback(sensor_msgs::JointState& joint_state)
{
  joint_state.header.stamp = ros::Time::now();

  for (auto& j : joints)
  {
    joint_state.name.push_back(j->name);
    joint_state.velocity.push_back(j->feedback);
  }

  return;
}

void JrkHardware::read(const ros::Time& time, const ros::Duration& period)
{
  for (auto& j : joints)
  {
    try
    {
      j->feedback = j->jrk.getFeedback();
      j->vel = fromArb(j->feedback);
      j->pos += j->vel * period.toSec();
    }
    catch (const jrk::JrkTimeout&)
    {
      handle_timeout(j->name, "getting feedback");
    }
  }
}

void JrkHardware::write(const ros::Time& time, const ros::Duration& period)
{
  // don't write commands if there is an error
  if (!active) return;

  for (auto& j : joints)
  {
    try
    {
      j->target = toArb(j->cmd);
      j->jrk.setTarget(j->target);
    } catch (const jrk::JrkTimeout&) {
      handle_timeout(j->name, "sending command");
    }
  }
}

inline uint16_t JrkHardware::toArb(double physical_units)
{
  int16_t v = (int16_t)std::floor(physical_units*conversion_factor) + 2048;
  // clamp TODO find a way to optomise this
  uint16_t arb = (v < 0) ? 0 : (4095 < v) ? 4095 : v;
  return arb;
}

inline double JrkHardware::fromArb(uint16_t arb_units)
{
  return ((double)arb_units - 2048.0)/conversion_factor;
}

} // namespace jrk
