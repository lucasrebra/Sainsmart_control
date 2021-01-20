#include <generic_robot_driver/generic_robot_driver.h>
// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace generic_robot_driver {

GenericRobotDriver::GenericRobotDriver(ros::NodeHandle &nh, ros::NodeHandle &nh_local)
    : name_("generic_robot_driver"), nh_(nh), nh_local_(nh, name_), arm_activated_(true) {
  // Read parameters through ros parameter server
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, nh_local_, "debug_mode", debug_mode_);
  error += !rosparam_shortcuts::get(name_, nh_local_, "depedented", depedented_);
  error += !rosparam_shortcuts::get(name_, nh_local_, "offset_for_depedented", offset_for_depedented_);
  rosparam_shortcuts::shutdownIfError(name_, error);
  nh_local_.param("servo_pins", servo_pins_, std::vector<int>{0, 1, 2, 3, 4, 5, 6});
  nh_local_.param("joint_lower", joint_lower_,
                  std::vector<float>{-1.5707, -1.5707, -1.5707, -1.5707, -1.5707, -1.5707, 0.0});
  nh_local_.param("servo_lower", servo_lower_,
                  std::vector<float>{1.5707, 1.5707, 1.5707, 1.5707, 1.5707, 1.5707, 0.02});
  nh_local_.param("joint_upper", joint_upper_, std::vector<float>{1600, 2000, -1440, 1800, 2000, 2000, 4000});
  nh_local_.param("servo_upper", servo_upper_, std::vector<float>{8200, 8300, 1440, 8200, 8300, 8300, 7500});
  nh_local_.param("directions_reverse", reverse_direction_,
                  std::vector<bool>{false, false, true, false, false, false, false});

}

GenericRobotDriver::~GenericRobotDriver() {}

bool GenericRobotDriver::initialize() {

  servo_command_.data.resize(16, -1);

  activate_arm_service_ = nh_.advertiseService("/activate_arm", &GenericRobotDriver::activate_arm_routine, this);
  servo_command_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Int32MultiArray>(nh_, "/command", 100));

  return true;
}

bool GenericRobotDriver::activate_arm_routine(arm_msgs::ActivateArm::Request &req,
                                              arm_msgs::ActivateArm::Response &res) {
  arm_activated_ = req.activate;
  res.success = true;
  return true;
}

void GenericRobotDriver::getJointPosition(std::vector<double> &joint_position) {

  // assume perfect execution if we have no feedback
  // e.g we read state equal to the command
  // joint_position[i] = joint_position_command[i] on the hardware interface
  return;
}

void GenericRobotDriver::writeJointCommand(const std::vector<double> &joint_command) {
  // use API of the robot
  this->transform_command(joint_command, servo_command_);
  // publish the command to a topic
  if (arm_activated_) {
    if (servo_command_pub_) {
      if (servo_command_pub_->trylock()) {
        servo_command_pub_->msg_ = servo_command_;
        servo_command_pub_->unlockAndPublish();
      }
    }
  }
}

void GenericRobotDriver::transform_command(const std::vector<double> &joint_command,
                                           std_msgs::Int32MultiArray &servo_command) {

  for (size_t i = 0; i < servo_pins_.size(); i++) {
    float output = this->map(joint_command[i], joint_lower_[i], joint_upper_[i], servo_lower_[i], servo_upper_[i],
                             reverse_direction_[i]);
    servo_command.data[servo_pins_[i]] = (int)output;
  }

  // if Depedented link
  // many diy robots have a close kinematic loop between the shoulder lift and elbow lift joints
  // in the urdf we assume that they are indepededent so here we need to do a trick
  // if we move servo 1 and servo 2 together the robot will move as if the servos are indepedent
  // servo 1 and servo2 probably are oriented in opposite manners so they will probably need to have
  // opposite directions
  // so we need to locate a helper variable that will move the servo 2 together with the 1
  // e.g servo 1 = 90 and servo 2 = 90 form orthogonal angle -> offset_for_depedented 0
  // then with e.g servo 1 = 110 and servo 2 = 70 will again form orthogonal angle
  // then servo 2 can move if servo 1 = 90 in  the range (50- 130)
  // so the lower and upper servo commands should be -40 , +40
  if (depedented_) {
    // helper : reverse of servo 1 + offset_for_depedented to get a position of servo 2 relative to servo 1
    int helper = int(servo_command.data[servo_pins_[1]] + offset_for_depedented_);
    // if we substitute this helper to the servo 2 the elbow must remain in a constant
    // angle with shoulder while the shoulder moves e.g we made them indepedent

    // servo_command.data[servo_pins[2]] = helper;

    // Notice how on the servo limits we have limits of the servo 2 relative to the position of servo 1

    // Now add to the servo 2 command the helper
    servo_command.data[servo_pins_[2]] += helper;
  }
}

float GenericRobotDriver::map(float inValue, float minInRange, float maxInRange, float minOutRange, float maxOutRange,
                              bool reverse_direction) {
  float x = (inValue - minInRange) / (maxInRange - minInRange);
  float result = minOutRange + (maxOutRange - minOutRange) * x;
  if (reverse_direction)
    return maxOutRange - result;
  return result;
}

} // namespace generic_robot_driver
