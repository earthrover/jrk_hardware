
#include <algorithm>
#include <exception>
#include <sstream>
#include <stdexcept>

#include "jrk_hardware/jrk_hardware.h"

using jrk::JrkHardware;
using std::size_t;

JrkHardware::JrkHardware(joints_t joints, double conversion_factor)
  : joints(joints)
  , num_joints(joints.size())
  , serial_devices(num_joints)
  , cmd(num_joints, 0.0)
  , pos(num_joints, 0.0)
  , vel(num_joints, 0.0)
  , eff(num_joints, 0.0)
  , conversion_factor(conversion_factor)
{
  if (num_joints < 1)
  {
    throw std::invalid_argument("JrkHardware requires at least 1 joint.");
  }

  createDevices(joints);
  registerInterfaces();

  active = true;
}

JrkHardware::~JrkHardware()
{
  stop();
}

void JrkHardware::registerInterfaces()
{
  size_t i = 0;
  for (const auto& j : joints)
  {
    hardware_interface::JointStateHandle joint_state_handle(j.first, &pos[i], &vel[i], &eff[i]);
    joint_state_interface.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd[i]);
    velocity_joint_interface.registerHandle(joint_handle);

    i++;
  }

  registerInterface(&joint_state_interface);
  registerInterface(&velocity_joint_interface);
}

void JrkHardware::createDevices(joints_t joints)
{
  size_t i = 0;
  for (const auto& j : joints)
  {
    serial_devices[i].setPort(j.second);
    serial_devices[i].setBaudrate(baudrate);
    serial_devices[i].setTimeout(timeout);
    serial_devices[i].open();

    jrk::Jrk jrk(serial_devices[i]);
    jrk_devices.push_back(jrk);

    i++;
  }
}

void JrkHardware::stop()
{
  active = false;
  for(size_t i = 0; i < num_joints; i++)
  {
    jrk_devices[i].motorOff();
  }
}

uint16_t JrkHardware::errors()
{
  try
  {
    for(auto& jrk : jrk_devices)
    {
      uint16_t err = jrk.getErrorsHalting();
      if (err && err != 1) // ignore the awaiting command error
      {
        stop();
        return err;
      }
    }

    return 0;
  }
  catch (const jrk::JrkTimeout&)
  {
    stop();
    return -1;
  }
}

void JrkHardware::read()
{
  for(size_t i = 0; i < num_joints; i++)
  {
    try
    {
      vel[i] = jrk_devices[i].getFeedback();
    }
    catch (const jrk::JrkTimeout&)
    {
      stop();
    }
  }

  return;
}

void JrkHardware::write()
{
  // don't write commands if there is an error
  if (!active) return;

  for (size_t i = 0; i < num_joints; i++)
  {
    jrk_devices[i].setTarget(toArb(cmd[i]));
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
  return ((double)arb_units-2048.0)/conversion_factor;
}

std::string JrkHardware::debug()
{
  std::stringstream ss;

  std::cout << num_joints << std::endl;
  ss << "Jrk Devices: [" << num_joints << "]{ ";

  for (auto j = joints.cbegin(); j != joints.cend(); ++j)
  {
    ss << j->first << " (" << j->second << ")";
    if (j != joints.cend()) ss << ", ";
  }
  ss << " }" << std::endl;

  return ss.str();
}
