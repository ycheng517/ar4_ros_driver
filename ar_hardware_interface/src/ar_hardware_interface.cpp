#include <ar_hardware_interface/ar_hardware_interface.hpp>
#include <sstream>

namespace ar_hardware_interface {

hardware_interface::CallbackReturn ARHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
  RCLCPP_INFO(logger_, "Initializing hardware interface...");

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  init_variables();
  // TODO: make these params in ar_ros2_control xacro:
  // loop_hz, joints, joint_limits, joint_offsets, encoder_steps_per_deg

  std::vector<double> enc_steps_per_deg = {-44.44444444, 55.55555556,
                                           55.55555556,  42.72664356,
                                           21.86024888,  22.22222222};

  // init motor driver
  std::string serial_port = "/dev/ttyACM0";
  int baudrate = 9600;
  driver_.init(serial_port, baudrate, num_joints_, enc_steps_per_deg);

  // set velocity limits
  velocity_limits_ = {0.34906585, 0.34906585, 0.34906585,
                      0.34906585, 0.34906585, 0.34906585};
  acceleration_limits_ = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  for (int i = 0; i < num_joints_; ++i) {
    velocity_limits_[i] = radToDeg(velocity_limits_[i]);
    acceleration_limits_[i] = radToDeg(acceleration_limits_[i]);
  }
  driver_.setStepperSpeed(velocity_limits_, acceleration_limits_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

void ARHardwareInterface::init_variables() {
  // TODO: get joint names properly
  joint_names_ = {"joint_1", "joint_2", "joint_3",
                  "joint_4", "joint_5", "joint_6"};
  num_joints_ = static_cast<int>(joint_names_.size());

  // resize vectors
  actuator_commands_.resize(num_joints_);
  actuator_positions_.resize(num_joints_);
  joint_positions_.resize(num_joints_);
  joint_velocities_.resize(num_joints_);
  joint_efforts_.resize(num_joints_);
  joint_position_commands_.resize(num_joints_);
  joint_velocity_commands_.resize(num_joints_);
  joint_effort_commands_.resize(num_joints_);
  joint_offsets_ = {170.0, -39.6, 0.0, -164.5, -104.5, -148.1};
  velocity_limits_.resize(num_joints_);
  acceleration_limits_.resize(num_joints_);
}

hardware_interface::CallbackReturn ARHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Activating hardware interface...");

  // calibrate joints if needed
  bool use_existing_calibrations = false;
  if (!use_existing_calibrations) {
    // run calibration
    RCLCPP_INFO(logger_, "Running joint calibration...");
    driver_.calibrateJoints();
  }

  // init position commands at current positions
  driver_.getJointPositions(actuator_positions_);
  for (int i = 0; i < num_joints_; ++i) {
    // apply offsets, convert from deg to rad for moveit
    joint_positions_[i] = degToRad(actuator_positions_[i] + joint_offsets_[i]);
    joint_position_commands_[i] = joint_positions_[i];
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ARHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Deactivating hardware interface...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ARHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto& joint_name : joint_names_) {
    state_interfaces.emplace_back(joint_name, "position",
                                  &joint_positions_[ind++]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ARHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  int ind = 0;
  for (const auto& joint_name : joint_names_) {
    command_interfaces.emplace_back(joint_name, "position",
                                    &joint_positions_[ind++]);
  }
  return command_interfaces;
}

hardware_interface::return_type ARHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  driver_.update(actuator_commands_, actuator_positions_);
  for (int i = 0; i < num_joints_; ++i) {
    // apply offsets, convert from deg to rad for moveit
    joint_positions_[i] = degToRad(actuator_positions_[i] + joint_offsets_[i]);
  }
  std::string logInfo = "Joint Position: \n";
  for (int i = 0; i < num_joints_; i++) {
    std::stringstream jointPositionStm;
    jointPositionStm << std::fixed << std::setprecision(3)
                     << radToDeg(joint_positions_[i]);
    logInfo += joint_names_[i] + ": " + jointPositionStm.str() + " | ";
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ARHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  for (int i = 0; i < num_joints_; ++i) {
    // convert from rad to deg, apply offsets
    actuator_commands_[i] =
        radToDeg(joint_position_commands_[i]) - joint_offsets_[i];
  }
  std::string logInfo = "Joint Position Command: \n";
  for (int i = 0; i < num_joints_; i++) {
    std::stringstream jointPositionStm;
    jointPositionStm << std::fixed << std::setprecision(3)
                     << radToDeg(joint_position_commands_[i]);
    logInfo += joint_names_[i] + ": " + jointPositionStm.str() + " | ";
  }
  return hardware_interface::return_type::OK;
}

}  // namespace ar_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ar_hardware_interface::ARHardwareInterface,
                       hardware_interface::SystemInterface)
