
#include <string>
#include <map>

#include "ros/ros.h"
#include "controller_manager/controller_manager.h"

#include "jrk_hardware/jrk_serial.h"
#include "jrk_hardware/jrk_hardware.h"

#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/JointState.h"

#include "diagnostic_msgs/DiagnosticArray.h"

#define JOINT_STATE_PUBLISH_FEEDBACK "/jrk_hardware/joints_raw_feedback"
//#define JOINT_STATE_PUBLISH_FEEDBACK "/joint_states"

void controlLoop(jrk::JrkHardware &jrk,
	controller_manager::ControllerManager &cm,
	ros::Rate control_rate,
	realtime_tools::RealtimePublisher<sensor_msgs::JointState>* feedback_pub)
{
	uint8_t counter = 0;
	ros::Time prev_time = ros::Time::now();

	while (ros::ok())
	{
		// Calculate monotonic time difference
		const ros::Time time = ros::Time::now();
		const ros::Duration elapsed = time - prev_time;

		// Process control loop
		const auto read_start = ros::Time::now();
		jrk.read(time, elapsed);

		cm.update(time, elapsed);

		const auto write_start = ros::Time::now();
		jrk.write(time, elapsed);

		if (feedback_pub)
		{
			if (feedback_pub->trylock())
			{
				jrk.raw_feedback(feedback_pub->msg_);
				feedback_pub->unlockAndPublish();
			}
			else
			{
				ROS_WARN("could not acquire lock, feedback is still publishing");
			}
		}

		// sleep
		control_rate.sleep();
	}

	// Stop the motors when the node closes
	jrk.stop();
}

int main(int argc, char *argv[])
{
	printf("------------------ MAIN JRK_HARDWARE_NODE " __DATE__ " " __TIME__ "-------------------\n");
	ros::init(argc, argv, "jrk_hardware_node");
	ros::NodeHandle nh, private_nh("~");

	auto diagnostics_pub = new realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray>(nh, "diagnostics", 10);

	double control_frequency, diagnostic_frequency, conversion_factor;

	double front_left_center, back_left_center;
	double front_right_center, back_right_center;

	private_nh.param<double>("control_frequency", control_frequency, 15.0);
	private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);
	private_nh.param<double>("conversion_factor", conversion_factor, 2048.0);

	// TODO: Read this properly and send it in a structure

	private_nh.param<double>("front_left_steering_center",  front_left_center, 2047);
	private_nh.param<double>("front_right_steering_center", front_right_center, 2047);
	private_nh.param<double>("back_left_steering_center",   back_left_center, 2047);
	private_nh.param<double>("back_right_steering_center",  back_right_center, 2047);

	double front_left_min, back_left_min;
	double front_right_min, back_right_min;

	private_nh.param<double>("front_left_steering_min",  front_left_min, 0.0);
	private_nh.param<double>("front_right_steering_min", front_right_min, 0.0);
	private_nh.param<double>("back_left_steering_min",   back_left_min, 0.0);
	private_nh.param<double>("back_right_steering_min",  back_right_min, 0.0);

	double front_left_max, back_left_max;
	double front_right_max, back_right_max;

	private_nh.param<double>("front_left_steering_max",  front_left_max, 4096);
	private_nh.param<double>("front_right_steering_max", front_right_max, 4096);
	private_nh.param<double>("back_left_steering_max",   back_left_max, 4096);
	private_nh.param<double>("back_right_steering_max",  back_right_max, 4096);

	bool publish_feedback;
	private_nh.param<bool>("publish_raw_feedback", publish_feedback, false);

	// Hardware steering will output the contents of the steering into a central device and packages will contain an ID for the hardware to understand
	bool hardware_pwm_steering;
	private_nh.param<bool>("hardware_pwm_steering", hardware_pwm_steering, false);

	int publish_feedback_rate;
	private_nh.param<int>("publish_feedback_rate", publish_feedback_rate, control_frequency);
	realtime_tools::RealtimePublisher<sensor_msgs::JointState>* feedback_pub;
	if (publish_feedback)
	{
		feedback_pub = new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh, JOINT_STATE_PUBLISH_FEEDBACK, 1); // Queue Size
	}

	std::map<std::string, std::string> joints;
	if (private_nh.hasParam("joints"))
	{
		private_nh.getParam("joints", joints);
	}
	else
	{
		ROS_FATAL("No joints found, cannot continue.");
		return -1;
	}

	ROS_DEBUG_STREAM("Creating JrkHardware with " << joints.size() << " joints.");

	jrk::JrkHardware jrk(joints, conversion_factor,
		front_left_center, back_left_center, front_right_center, back_right_center,
		front_left_max, back_left_max, front_right_max, back_right_max,
		front_left_min, back_left_min, front_right_min, back_right_min,
		hardware_pwm_steering);

	controller_manager::ControllerManager cm(&jrk, nh);

	printf(" Conversion factor %2.2f \n", conversion_factor);

	ros::Rate control_rate(control_frequency);

	// start the ros spinner
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// run the control loop
	ROS_INFO_STREAM("Starting the jrk_hardware control loop.");
	if (publish_feedback)
	{
		ROS_INFO_STREAM("USING FEEDBACK LOOP");
		controlLoop(jrk, cm, control_rate, feedback_pub);
	}
	else
	{
		controlLoop(jrk, cm, control_rate, nullptr);
	}

	return 0;
}
