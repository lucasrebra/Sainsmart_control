#include <ros/ros.h>

#include <generic_hardware_interface/generic_hw_control_loop.h>
#include <generic_hardware_interface/generic_hw_interface.h>
#include <controller_manager/controller_manager.h>
#include <csignal>

// hardware interface pointer
boost::shared_ptr<generic_hardware_interface::GenericHardwareInterface> hw_interface_ptr;
// control loop pointer
boost::shared_ptr<generic_hardware_control_loop::GenericHWControlLoop> hw_control_loop_ptr;

// Interrupt signal
void signalHandler(int signum) {

  ROS_WARN_STREAM("[generic_hw_interface] Interrupt signal (" << signum << ") received.\n");

  hw_interface_ptr.reset();
  hw_control_loop_ptr.reset();

  exit(signum);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "generic_hardware_interface");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);
  // Create the hardware interface
  hw_interface_ptr.reset(new generic_hardware_interface::GenericHardwareInterface);
  if (!hw_interface_ptr->init(nh, nh_local)) {
    ROS_ERROR_STREAM("[generic_hw_interface_node] Could not correctly initialize robot. Exiting");
    exit(1);
  }
  ROS_INFO_STREAM("[generic_hw_interface_node] HW interface initialized");
  // Start the control loop
  hw_control_loop_ptr.reset(new generic_hardware_control_loop::GenericHWControlLoop(nh, hw_interface_ptr));
  hw_control_loop_ptr->run(); // Blocks until shutdown signal received

  return 0;
}
