#include <sstream>

#include <generic_hardware_interface/generic_hw_interface.h>
// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace generic_hardware_interface {

GenericHardwareInterface::GenericHardwareInterface() : position_controller_running_(true) {}

bool GenericHardwareInterface::init(ros::NodeHandle &nh, ros::NodeHandle &nh_local) {
  // Get Generic Hardware Interface parameters
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, nh_local, "joints", joint_names_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // Resize vectors
  num_joints_ = joint_names_.size();
  joint_position_.resize(num_joints_, 0.0);
  joint_velocity_.resize(num_joints_, 0.0);
  joint_effort_.resize(num_joints_, 0.0);

  joint_position_command_.resize(num_joints_, 0.0);

  // Initialize Generic Robot driver
  robot_driver_ptr_.reset(new generic_robot_driver::GenericRobotDriver(nh_, nh_local_));
  bool robot_driver_initialized = robot_driver_ptr_->initialize();
  if (!robot_driver_initialized) {
    ROS_ERROR_STREAM("[generic_hw_interface] Failed to initialize robot driver");
    return false;
  }

  // Create ros_control interfaces
  for (size_t i = 0; i < num_joints_; ++i) {
    ROS_DEBUG_STREAM("[generic_hw_interface] Registering handles for joint " << joint_names_[i]);
    try {
      // Create joint state interface
      joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
          joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
      // Create position joint interface
      position_joint_interface_.registerHandle(hardware_interface::JointHandle(
          joint_state_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));
    } catch (const hardware_interface::HardwareInterfaceException &e) {
      ROS_ERROR_STREAM("[generic_hw_interface] " << e.what());
      return false;
    }
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
  return true;
}

void GenericHardwareInterface::read(const ros::Time &time, const ros::Duration &period) {
  // assume perfect execution pass command into state
  // uncomment if you have real feedback
  for(size_t i=0; i<joint_position_command_.size(); i++){
    joint_position_[i] = joint_position_command_[i];
  }

  // will just return
  if (position_controller_running_) {
    robot_driver_ptr_->getJointPosition(joint_position_);
  }
  
}

void GenericHardwareInterface::write(const ros::Time &time, const ros::Duration &period) {
  if (position_controller_running_) {
    robot_driver_ptr_->writeJointCommand(joint_position_command_);
  }
}

} // namespace generic_hardware_interface
