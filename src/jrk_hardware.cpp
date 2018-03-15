
#include <algorithm>
#include <exception>
#include <sstream>
#include <stdexcept>
#include "ros/ros.h"

#include "jrk_hardware/jrk_hardware.h"

namespace jrk
{
	const char *get_robot_name() {
		return "BOB";
	}

	JrkHardware::JrkHardware(std::map<std::string, std::string> joint_params, double conversion_factor)
		: joints()
		, conversion_factor(conversion_factor)
	{
		printf("+++++++++++ BUILDING ROBOT HARDWARE 0x%p +++++++++++\n", this);

		if (joint_params.size() < 1)
		{
			throw std::invalid_argument("JrkHardware requires at least 1 joint.");
		}

		for (const auto& params : joint_params)
		{
			try {
				printf("+ Build joint [%s] [%s]\n", params.first.c_str(), params.second.c_str());
				ROS_DEBUG_STREAM("creating joint " << params.first << " @ " << params.second);
				joints.emplace_back(new Joint(params.first, params.second));
			}
			catch (const std::exception &e) {
				printf("! Hardware Exception opening [%s] => [%s]\n", params.first.c_str(), params.second.c_str());
				ROS_FATAL("Serial port failed %s cannot continue. %s ", params.second.c_str(), e.what());
				throw e;
			}
		}

		printf("+ Register interfaces \n");
		registerInterfaces();

		active = true;
	}

	JrkHardware::~JrkHardware()
	{
		stop();
	}

	void JrkHardware::registerInterfaces()
	{
		/*
		std::vector<std::string> velocity_joints_name = {
			"front_left_wheel_joint", "front_right_wheel_joint",
			"back_left_wheel_joint", "back_right_wheel_joint" };

		// Connect and register the joint state and velocity interface
		for (unsigned int i = 0; i < velocity_joints_name.size(); ++i)
		{

			hardware_interface::JointStateHandle state_handle(velocity_joints_name[i], &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
			joint_state_interface.registerHandle(state_handle);

			hardware_interface::JointHandle vel_handle(state_handle, &joints_[i].velocity_command);
			position_joint_interface.registerHandle(vel_handle);
		}

		std::vector<std::string> position_joints_name = {
			"front_left_steering_joint", "front_right_steering_joint",
			"back_left_steering_joint", "back_right_steering_joint" };

		// Connect and register the joint state and position interface
		for (unsigned int i = 0; i < position_joints_name.size(); ++i)
		{
			hardware_interface::JointStateHandle state_handle(position_joints_name[i], &steering_joints_[i].position, &steering_joints_[i].velocity, &steering_joints_[i].effort);
			joint_state_interface.registerHandle(state_handle);

			hardware_interface::JointHandle pos_handle(state_handle, &steering_joints_[i].position_command);
			position_joint_interface.registerHandle(pos_handle);
		}

		printf("+ Joint State Interface 0x%p \n", &joint_state_interface);
		registerInterface(&joint_state_interface);

		printf("+ Velocity Interface 0x%p \n", &velocity_joint_interface);
		registerInterface(&velocity_joint_interface);


		printf("+ Position Interface 0x%p \n", &position_joint_interface);
		registerInterface(&position_joint_interface);
		*/

		for (auto& j : joints)
		{
			printf("+ Registering Interfaces %s\n", j->name.c_str());
			hardware_interface::JointStateHandle joint_state_handle(j->name, &j->pos, &j->vel, &j->eff);
			joint_state_interface.registerHandle(joint_state_handle);

			hardware_interface::JointHandle joint_handle_velocity(joint_state_handle, &j->cmd);
			velocity_joint_interface.registerHandle(joint_handle_velocity);

			// Handle to control the joint position
			// following setup at [https://github.com/ros-controls/ros_control/wiki/hardware_interface]
			hardware_interface::JointHandle joint_handle_position(joint_state_handle, &j->cmd);
			position_joint_interface.registerHandle(joint_handle_position);
		}

		registerInterface(&joint_state_interface);
		registerInterface(&velocity_joint_interface);
		registerInterface(&position_joint_interface);
	}

	void JrkHardware::stop()
	{
		if (!active) return;
		active = false;
		for (auto& j : joints)
		{
			try {
				j->jrk.motorOff();
			}
			catch (const jrk::JrkTimeout&) {
				handle_timeout(j->name, "sending stop");
			}
		}
		ROS_WARN_STREAM("sending commands to motors has been disabled");
	}

	void JrkHardware::handle_timeout(const std::string joint_name, std::string details)
	{
		ROS_ERROR_STREAM("joint " << joint_name << " timed out " << details);
		//stop();
	}

	void JrkHardware::clear_errors()
	{
		for (auto& j : joints)
		{
			try {
				j->jrk.setTarget(2048);
			}
			catch (const jrk::JrkTimeout&) {
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
		uint8_t t = 0;
		bool debug_output = false;

		for (auto& j : joints)
		{
			try
			{
				j->feedback = j->jrk.getFeedback();
				j->vel = fromArb(j->feedback);
				j->pos += j->vel * period.toSec();

				float feed = abs(joints_[t].feedback - j->feedback);

				// We have some sensors not connected while testing so lets filter floating sensors
				if (feed > 50) {
					debug_output = true;
					joints_[t].feedback = j->feedback;
					joints_[t].velocity = j->vel;
					joints_[t].position = j->pos;
				}
			}
			catch (const jrk::JrkTimeout&)
			{
				handle_timeout(j->name, "getting feedback");
			}

			t++;
		}

		if (debug_output) {
			printf("------------- READ --------------\n");

			for (auto& j : joints)
			{
				printf("[%s] Feedback=[%d] Vel=[%2.2f] Pos=[%2.2f]\n", j->name.c_str(), j->feedback, j->vel, j->pos);
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
			}
			catch (const jrk::JrkTimeout&) {
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
		return ((double)arb_units - 2048.0) / conversion_factor;
	}

} // namespace jrk
