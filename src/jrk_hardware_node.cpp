
#include <string>
#include <map>

#include "ros/ros.h"
#include "controller_manager/controller_manager.h"

#include "jrk_hardware/jrk_serial.h"
#include "jrk_hardware/jrk_hardware.h"

void controlLoop(jrk::JrkHardware &jrk,
                 controller_manager::ControllerManager &cm,
                 ros::Rate rate)
{
  ros::Time prev_time = ros::Time::now();

  while(ros::ok())
  {
    // Calculate monotonic time difference
    const ros::Time time = ros::Time::now();
    const ros::Duration elapsed = time - prev_time;

    // Process control loop
    uint16_t err = jrk.errors();
    if (err)
    {
      ROS_WARN_STREAM("Motor error: " << jrk::readErrorFlags(err));
    }
    jrk.read();
    cm.update(time, elapsed);
    jrk.write();

    prev_time = time;

    rate.sleep();
  }

  // Stop the motors when the node closes
  jrk.stop();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "jrk_hardware_node");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency, conversion_factor;
  private_nh.param<double>("control_frequency", control_frequency, 10.0);
  private_nh.param<double>("conversion_factor", conversion_factor, 2048.0);

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

  ros::Rate rate(control_frequency);

  // start the ros spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // run the control loop
  ROS_INFO_STREAM("Starting the jrk_hardware control loop.");
  controlLoop(jrk, cm, rate);

  return 0;
}
