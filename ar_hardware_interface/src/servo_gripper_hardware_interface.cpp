#include <ar_hardware_interface/servo_gripper_hardware_interface.hpp>
#include <sstream>

namespace ar_gripper_hardware_interface {

hardware_interface::CallbackReturn ARGripperHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
  RCLCPP_INFO(logger_, "Initializing hardware interface...");

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;

  std::string serial_port = info_.hardware_parameters.at("serial_port");
  int baud_rate = 115200;
  bool success = driver_.init(serial_port, baud_rate);
  if (!success) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ARGripperHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Activating hardware interface...");

  // initialize gripper position
  position_ = driver_.getPosition();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ARGripperHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Deactivating hardware interface...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ARGripperHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  RCLCPP_INFO(logger_, "Debug: Exporting state interfaces for joint %s",
              info_.joints[0].name.c_str());
  state_interfaces.emplace_back(info_.joints[0].name, "position", &position_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ARGripperHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(info_.joints[0].name, "position",
                                  &position_command_);
  return command_interfaces;
}

hardware_interface::return_type ARGripperHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  position_ = driver_.getPosition();
  std::string logInfo = "Gripper Pos: " + std::to_string(position_);
  RCLCPP_INFO_THROTTLE(logger_, clock_, 500, logInfo.c_str());
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ARGripperHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  std::string logInfo = "Gripper Cmd: " + std::to_string(position_command_);
  RCLCPP_INFO_THROTTLE(logger_, clock_, 500, logInfo.c_str());
  driver_.writePosition(position_command_);
  return hardware_interface::return_type::OK;
}

}  // namespace ar_gripper_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ar_gripper_hardware_interface::ARGripperHardwareInterface,
    hardware_interface::SystemInterface)
