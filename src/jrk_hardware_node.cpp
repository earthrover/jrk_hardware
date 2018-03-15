
#include <string>
#include <map>

#include "ros/ros.h"
#include "controller_manager/controller_manager.h"

#include "jrk_hardware/jrk_serial.h"
#include "jrk_hardware/jrk_hardware.h"

#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/JointState.h"

#include "diagnostic_msgs/DiagnosticArray.h"

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
	printf("------------------ MAIN JRK_HARDWARE_NODE -------------------\n");
	ros::init(argc, argv, "jrk_hardware_node");
	ros::NodeHandle nh, private_nh("~");

	auto diagnostics_pub = new realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray>(nh, "diagnostics", 10);

	double control_frequency, diagnostic_frequency, conversion_factor;
	private_nh.param<double>("control_frequency", control_frequency, 10.0);
	private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);
	private_nh.param<double>("conversion_factor", conversion_factor, 2048.0);

	bool publish_feedback;
	private_nh.param<bool>("publish_raw_feedback", false);
	realtime_tools::RealtimePublisher<sensor_msgs::JointState>* feedback_pub;
	if (publish_feedback)
	{
		feedback_pub = new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh, "raw_feedback", 10);
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

	jrk::JrkHardware jrk(joints, conversion_factor);
	controller_manager::ControllerManager cm(&jrk, nh);

	printf("------------------ RobotHW 0x%p -------------------\n", &jrk);
	printf("------------- ControllerManager 0x%p -------------------\n", &cm);

	ros::Rate control_rate(control_frequency);

	// start the ros spinner
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// run the control loop
	ROS_INFO_STREAM("Starting the jrk_hardware control loop.");
	if (publish_feedback)
	{
		controlLoop(jrk, cm, control_rate, feedback_pub);
	}
	else
	{
		controlLoop(jrk, cm, control_rate, nullptr);
	}

	return 0;
}
