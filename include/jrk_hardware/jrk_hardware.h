
#ifndef JRK_CONTROL_HARDWARE_H
#define JRK_CONTROL_HARDWARE_H

#include <map>
#include <string>
#include <vector>

#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"

#include "sensor_msgs/JointState.h"

#include "serial/serial.h"

#include "jrk_hardware/jrk_serial.h"

#define MAX_MOTORS 16

namespace jrk
{
	class JrkHardware : public hardware_interface::RobotHW
	{
		struct JointWheel
		{
			double position;
			double velocity;
			double effort;
			int feedback;
			double velocity_command;

			JointWheel() : position(0), velocity(0), effort(0), velocity_command(0) { }
		} joints_[4];

		struct SteeringJoint
		{
			double position;
			double velocity;
			double effort;
			int feedback;
			double position_command;

			SteeringJoint() : position(0), velocity(0), effort(0), position_command(0) { }
		} steering_joints_[4];

	public:

		JrkHardware(std::map<std::string, std::string> joints, double conversion_factor);
		virtual ~JrkHardware();

		void read(const ros::Time& time, const ros::Duration& period) override;
		void write(const ros::Time& time, const ros::Duration& period) override;

		void raw_feedback(sensor_msgs::JointState& joint_state);

		void stop();
		void clear_errors();

		const char *get_robot_name();

	private:
		void registerInterfaces();
		void createDevices(std::map<std::string, std::string> joints);

		void handle_timeout(std::string joint_name, std::string details = "");

		inline uint16_t toArb(double physical_units);
		inline double fromArb(uint16_t arb_units);

		hardware_interface::JointStateInterface joint_state_interface;
		hardware_interface::VelocityJointInterface velocity_joint_interface;
		hardware_interface::PositionJointInterface position_joint_interface;

		struct Joint
		{
			const std::string name;
			const std::string port;
			jrk::Jrk jrk;

			uint16_t target, feedback, error;
			double cmd, pos, vel, eff;

			Joint(const std::string name = "", const std::string port = "", const unsigned long baudrate = 9600, const uint32_t timeout = 500)
				: name(name), port(port), jrk(port, baudrate, timeout)
				, target(2048), feedback(2048), error(1), cmd(0), pos(0), vel(0), eff(0)
			{ }
		};

		std::vector<std::unique_ptr<Joint> > joints;

		double conversion_factor;

		bool active;
	};
}  // namespace jrk_control

#endif  // JRK_CONTROL_HARDWARE_H
