
#include <algorithm>
#include <exception>
#include <sstream>
#include <stdexcept>
#include "ros/ros.h"

#include "jrk_hardware/jrk_hardware.h"

using namespace jrk;

// Enable printf constant debug on this file
// #define DEBUG

const char *get_robot_name() {
	return "BOB";
}

int rotate_in_place = 0;

jrk::Jrk *hardware_steering = NULL;

jrk::JrkHardware::Joint::Joint(const std::string name, const std::string port, const double min_pos, const double max_pos, const double center_pos,
	const unsigned long baudrate, const uint32_t timeout)
	: name(name), port(port)
	, target(2048), feedback(2048), error(1), cmd(0), pos(0), vel(0), eff(0), min_(min_pos), max_(max_pos), center_(center_pos), id_hardware(0), is_pololu(true)
{
	size_t len = port.length();
	const char *prt = port.c_str();

	// We are loooking for the hardware ID to specify if we are going to pack the packages into one.
	if (prt[len - 2] == '&') {
		std::string new_port = port.substr(0, len - 2);
		std::string id = port.substr(len - 1, 1);
		printf(" Port %s ID %s \n", new_port.c_str(), id.c_str());

		if (hardware_steering == NULL)
		{
			printf(" New port ");
            jrk = new jrk::Jrk(new_port, 115200, timeout);
		}
		else {
			printf(" Reusing port %s \n", new_port.c_str());
			jrk = hardware_steering;
		}
	}
	else {
		jrk = new jrk::Jrk(port, baudrate, timeout);
	}
}

JrkHardware::JrkHardware(std::map<std::string, std::string> joint_params, double conversion_factor,
	float front_left_center, float back_left_center, float front_right_center, float back_right_center,
	float front_left_max, float back_left_max, float front_right_max, float back_right_max,
	float front_left_min, float back_left_min, float front_right_min, float back_right_min,
	bool hardware_pwm_steering)
	: joints()
	, conversion_factor(conversion_factor)
{
	printf("+++++++++++ BUILDING ROBOT HARDWARE +++++++++++\n");

	if (joint_params.size() < 1)
	{
		throw std::invalid_argument("JrkHardware requires at least 1 joint.");
	}

	for (const auto& params : joint_params)
	{
		try {
			double min_ = 0;
			double max_ = 4096;
			double center_ = 2048;

			if (hardware_pwm_steering) {

			}

			printf("+ Build joint [%s] [%s]\n", params.first.c_str(), params.second.c_str());
			ROS_DEBUG_STREAM("creating joint " << params.first << " @ " << params.second);

			if (!strcmp(params.first.c_str(), "front_left_steering_joint")) {
				min_ = front_left_min;
				max_ = front_left_max;
				center_ = front_left_center;
				printf("+ Front Left MIN %2.0f MAX %2.0f, CENTER %2.0f \n", min_, max_, center_);
			}

			if (!strcmp(params.first.c_str(), "back_left_steering_joint")) {
				min_ = back_left_min;
				max_ = back_left_max;
				center_ = back_left_center;
				printf("+ Back Left MIN %2.0f MAX %2.0f, CENTER %2.0f \n", min_, max_, center_);
			}

			if (!strcmp(params.first.c_str(), "front_right_steering_joint")) {
				min_ = front_right_min;
				max_ = front_right_max;
				center_ = front_right_center;
				printf("+ Front Right MIN %2.0f MAX %2.0f, CENTER %2.0f \n", min_, max_, center_);
			}

			if (!strcmp(params.first.c_str(), "back_right_steering_joint")) {
				min_ = back_right_min;
				max_ = back_right_max;
				center_ = back_right_center;
				printf("+ Back Right MIN %2.0f MAX %2.0f, CENTER %2.0f \n", min_, max_, center_);
			}

			Joint *joint = new Joint(params.first, params.second, min_, max_, center_);
			joints.emplace_back(joint);
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
	for (auto& j : joints)
	{
		printf("+ Registering Interfaces %s\n", j->name.c_str());
		hardware_interface::JointStateHandle joint_state_handle(j->name, &j->pos, &j->vel, &j->eff);
		joint_state_interface.registerHandle(joint_state_handle);

		if (strstr(j->name.c_str(), "steering") == NULL) {
			hardware_interface::JointHandle joint_handle_velocity(joint_state_handle, &j->cmd);
			velocity_joint_interface.registerHandle(joint_handle_velocity);
		}
		else {
			// Handle to control the joint position
			// following setup at [https://github.com/ros-controls/ros_control/wiki/hardware_interface]
			hardware_interface::JointHandle joint_handle_position(joint_state_handle, &j->cmd);
			position_joint_interface.registerHandle(joint_handle_position);
		}
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
			j->jrk->motorOff();
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
			j->jrk->setTarget(2048);
		}
		catch (const jrk::JrkTimeout&) {
			handle_timeout(j->name, "clearing errors");
		}
		// there should be no errors now, otherwise exit
		if ((j->error = j->jrk->getErrorsHalting()))
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

#ifdef DEBUG
	printf("-------------- Publish %d -------------------\n", ros::Time::now().sec);
#endif

	joint_state.name.clear();
	joint_state.position.clear();
	joint_state.velocity.clear();
	joint_state.effort.clear();

	for (auto& j : joints)
	{
#ifdef DEBUG
		printf("   [%s] Pos -> %2.2f \n", j->name.c_str(), j->pos);
#endif
		joint_state.name.push_back(j->name);

		if (strstr(j->name.c_str(), "steering") == NULL) {
			joint_state.position.push_back(0);
			joint_state.velocity.push_back(j->vel);
		}
		else {
			joint_state.position.push_back(fromArb(j->feedback));
			joint_state.velocity.push_back(j->vel);
		}

		joint_state.effort.push_back(0);
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
			j->feedback = j->jrk->getFeedback();

			if (strstr(j->name.c_str(), "steering") == NULL) {
				j->vel = fromArb(j->feedback);
				j->pos += j->vel * period.toSec();
			}

			joints_[t].velocity = j->vel;
			joints_[t].position = j->pos;
			joints_[t].feedback = j->feedback;

			// We have some sensors not connected while testing so lets filter floating sensors
#ifdef DEBUG
			float feed = abs(joints_[t].feedback - j->feedback);
			if (feed > 20) {
				debug_output = true;
			}
#endif
		}
		catch (const jrk::JrkTimeout&)
		{
			handle_timeout(j->name, "getting feedback");
		}

		t++;
	}

#ifdef DEBUG
	if (debug_output) {
		printf("------------- READ --------------\n");

		FILE * fp;
		fp = fopen("/home/earth/earth_read.txt", "w+");

		for (auto& j : joints)
		{
			printf("[%10s] Feedback=[%d] Vel=[%2.2f] Pos=[%2.2f]\n", j->name.c_str(), j->feedback, j->vel, j->pos);
			if (fp) {
				fprintf(fp, "[%10s] Feedback=[%d] Vel=[%2.2f] Pos=[%2.2f]\n", j->name.c_str(), j->feedback, j->vel, j->pos);
			}
		}

		fclose(fp);
	}
#endif

}

void JrkHardware::write(const ros::Time& time, const ros::Duration& period)
{
	static unsigned int value = 0;

	// don't write commands if there is an error
    if (!active) {
        printf("!!!ERROR!!!\n");
        return;
    }

	bool output_debug = false;

	if ((value % 1) == 0) {
        // Hack debugging, we create files on disk for different purposes
        if (access("/home/earth/debug.txt", F_OK) != -1)
            output_debug = true;
        else
            output_debug = false;

		if (access("/home/earth/rotate.txt", F_OK) != -1) {
			if (rotate_in_place == 0)
				printf("!!! ROTATE !!!\n");
			rotate_in_place = 1;
		} else {
			rotate_in_place = 0;
		}
	}

	for (auto& j : joints)
	{
		try
		{
			if (strstr(j->name.c_str(), "steering") == NULL) {
				j->target = toArb(j->cmd / 8.33f);

				if (rotate_in_place != 0) {
					if (strstr(j->name.c_str(), "right")) {
						j->target = 4095 - j->target;
					}
				}

			}
			else {

				if (rotate_in_place != 0) {
					if (!strcmp(j->name.c_str(), "front_left_steering_joint")) {
						j->target = j->min_;
					}
					else

						if (!strcmp(j->name.c_str(), "back_left_steering_joint")) {
							j->target = j->max_;
						}
						else

							if (!strcmp(j->name.c_str(), "front_right_steering_joint")) {
								j->target = j->max_;
							}
							else

								if (!strcmp(j->name.c_str(), "back_right_steering_joint")) {
									j->target = j->min_;
								}

				}
				else {
					j->target = toArb(j->cmd);
					j->target -= conversion_factor - j->center_;

					if (j->target < j->min_)
						j->target = j->min_;

					if (j->target > j->max_)
						j->target = j->max_;
				}

			}

            if (j->target != 2048) {
                output_debug = true;
            }

#ifdef DEBUG_PWM
            printf(" Set target %s\n", j->name.c_str());
#endif
			j->jrk->setTarget(j->target);

		} catch (const jrk::JrkTimeout&) {
			handle_timeout(j->name, "sending command");
		}
	}

	if (output_debug && (value % 100) == 0) {
		printf("------------- WRITE %d --------------\n", value);

		for (auto& j : joints)
		{
			printf("[%10s] Target [%d] [%2.2f] \n", j->name.c_str(), j->target, j->cmd);
		}
	}

	value++;
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

