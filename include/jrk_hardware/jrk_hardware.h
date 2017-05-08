
#ifndef JRK_CONTROL_HARDWARE_H
#define JRK_CONTROL_HARDWARE_H

#include <map>
#include <string>
#include <vector>

#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"

#include "serial/serial.h"

#include "jrk_hardware/jrk_serial.h"

namespace jrk
{

typedef std::map<std::string, std::string> joints_t;

class JrkHardware : public hardware_interface::RobotHW
{
public:
  JrkHardware(joints_t joints, double conversion_factor);
  virtual ~JrkHardware();

  uint16_t errors();
  void read();
  void write();
  void stop();
  std::string debug();

private:
  void registerInterfaces();
  void createDevices(joints_t joints);

  inline uint16_t toArb(double physical_units);
  inline double fromArb(uint16_t arb_units);

  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::VelocityJointInterface velocity_joint_interface;

  serial::Timeout timeout = serial::Timeout::simpleTimeout(500);
  const unsigned long baudrate = 9600;

  size_t num_joints;
  joints_t joints;
  std::vector<serial::Serial> serial_devices;
  std::vector<jrk::Jrk> jrk_devices;

  std::vector<double> cmd;
  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<double> eff;

  double conversion_factor;

  bool active;
};
}  // namespace jrk_control

#endif  // JRK_CONTROL_HARDWARE_H
