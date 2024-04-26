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

  info_ = info;
  init_variables();

  // init motor driver
  std::string serial_port = info_.hardware_parameters.at("serial_port");
  int baud_rate = 9600;
  driver_.init(serial_port, baud_rate, info_.joints.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

void ARHardwareInterface::init_variables() {
  // resize vectors
  int num_joints = info_.joints.size();
  actuator_commands_.resize(num_joints);
  actuator_positions_.resize(num_joints);
  joint_positions_.resize(num_joints);
  joint_velocities_.resize(num_joints);
  joint_efforts_.resize(num_joints);
  joint_position_commands_.resize(num_joints);
  joint_velocity_commands_.resize(num_joints);
  joint_effort_commands_.resize(num_joints);
  joint_offsets_.resize(num_joints);
  for (int i = 0; i < num_joints; ++i) {
    joint_offsets_[i] =
        std::stod(info_.joints[i].parameters["position_offset"]);
  }
}

hardware_interface::CallbackReturn ARHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Activating hardware interface...");

  // calibrate joints if needed
  bool calibrate = info_.hardware_parameters.at("calibrate") == "True";
  if (calibrate) {
    // run calibration
    RCLCPP_INFO(logger_, "Running joint calibration...");
    driver_.calibrateJoints();
  }

  // init position commands at current positions
  driver_.getJointPositions(actuator_positions_);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
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

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, "position",
                                  &joint_positions_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ARHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, "position",
                                    &joint_position_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type ARHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  driver_.getJointPositions(actuator_positions_);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // apply offsets, convert from deg to rad for moveit
    joint_positions_[i] = degToRad(actuator_positions_[i] + joint_offsets_[i]);
  }
  std::string logInfo = "Joint Pos: ";
  for (size_t i = 0; i < info_.joints.size(); i++) {
    std::stringstream jointPositionStm;
    jointPositionStm << std::fixed << std::setprecision(2)
                     << radToDeg(joint_positions_[i]);
    logInfo += info_.joints[i].name + ": " + jointPositionStm.str() + " | ";
  }
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ARHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // convert from rad to deg, apply offsets
    actuator_commands_[i] =
        radToDeg(joint_position_commands_[i]) - joint_offsets_[i];
  }
  std::string logInfo = "Joint Cmd: ";
  for (size_t i = 0; i < info_.joints.size(); i++) {
    std::stringstream jointPositionStm;
    jointPositionStm << std::fixed << std::setprecision(2)
                     << radToDeg(joint_position_commands_[i]);
    logInfo += info_.joints[i].name + ": " + jointPositionStm.str() + " | ";
  }
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());
  driver_.update(actuator_commands_, actuator_positions_);
  return hardware_interface::return_type::OK;
}

}  // namespace ar_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ar_hardware_interface::ARHardwareInterface,
                       hardware_interface::SystemInterface)
