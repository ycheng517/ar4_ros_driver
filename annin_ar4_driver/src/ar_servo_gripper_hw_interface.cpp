#include <algorithm>
#include <annin_ar4_driver/ar_servo_gripper_hw_interface.hpp>
#include <sstream>

#include "annin_ar4_driver/gripper_overcurrent_protection.hpp"

namespace annin_ar4_driver {

hardware_interface::CallbackReturn ARServoGripperHWInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
  RCLCPP_INFO(logger_, "Initializing hardware interface...");

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;

  // Extract position limits from robot description
  if (!info_.limits.empty()) {
    for (const auto& limit_pair : info_.limits) {
      if (limit_pair.first == "gripper_jaw1_joint") {
        if (limit_pair.second.has_position_limits) {
          closed_position_ = limit_pair.second.min_position;
          open_position_ = limit_pair.second.max_position;
          RCLCPP_INFO(logger_, "Using joint limits: closed = %f m, open = %f m",
                      closed_position_, open_position_);
        }
      }
    }
  }

  if (closed_position_ == 0.0 && open_position_ == 0.0) {
    RCLCPP_ERROR(logger_, "No joint limits found for gripper_jaw1_joint.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Configure overcurrent protection (if enabled)
  if (info_.hardware_parameters.count("use_overcurrent_protection") > 0) {
    std::string overcurrent_protection =
        info_.hardware_parameters.at("use_overcurrent_protection");
    std::transform(overcurrent_protection.begin(), overcurrent_protection.end(),
                   overcurrent_protection.begin(), ::tolower);
    if (overcurrent_protection == "true") {
      RCLCPP_INFO(logger_, "Overcurrent protection enabled for gripper");
      overcurrent_protection_ =
          std::make_unique<GripperOverCurrentProtection>(logger_, clock_);
    }
  }

  // Load servo angle parameters
  if (info_.hardware_parameters.count("closed_servo_angle") > 0) {
    closed_servo_angle_ =
        std::stoi(info_.hardware_parameters.at("closed_servo_angle"));
    RCLCPP_INFO(logger_, "Loaded closed_servo_angle: %d", closed_servo_angle_);
  } else {
    RCLCPP_ERROR(logger_, "Required parameter 'closed_servo_angle' not found");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.count("open_servo_angle") > 0) {
    open_servo_angle_ =
        std::stoi(info_.hardware_parameters.at("open_servo_angle"));
    RCLCPP_INFO(logger_, "Loaded open_servo_angle: %d", open_servo_angle_);
  } else {
    RCLCPP_ERROR(logger_, "Required parameter 'open_servo_angle' not found");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate servo angle range
  if (closed_servo_angle_ >= open_servo_angle_) {
    RCLCPP_ERROR(
        logger_,
        "Invalid servo angle range: min (%d) must be less than max (%d)",
        closed_servo_angle_, open_servo_angle_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string serial_port = info_.hardware_parameters.at("serial_port");
  int baud_rate = 115200;
  bool success = driver_.init(serial_port, baud_rate);
  if (!success) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ARServoGripperHWInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Activating hardware interface...");

  // Write ACS712 version if specified
  if (overcurrent_protection_ && info_.hardware_parameters.count("ACS712_version_current") > 0) {
    int current_version =
        std::stoi(info_.hardware_parameters.at("ACS712_version_current"));
    RCLCPP_INFO(logger_, "Setting ACS712 version to %dA", current_version);
    if (!driver_.writeACS712Version(current_version)) {
      RCLCPP_ERROR(logger_, "Failed to set ACS712 version");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // initialize gripper position
  int pos_deg;
  bool success = driver_.getPosition(pos_deg);
  if (!success) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  position_ = servo_angle_to_linear_pos(pos_deg);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ARServoGripperHWInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Deactivating hardware interface...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ARServoGripperHWInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, "position", &position_);
    state_interfaces.emplace_back(info_.joints[i].name, "velocity", &velocity_);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ARServoGripperHWInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, "position",
                                    &position_command_);
  }
  return command_interfaces;
}

hardware_interface::return_type ARServoGripperHWInterface::read(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  int pos_deg;
  bool success = driver_.getPosition(pos_deg);
  if (!success) {
    RCLCPP_ERROR(logger_, "Failed to read position from servo");
    return hardware_interface::return_type::ERROR;
  }
  position_ = servo_angle_to_linear_pos(pos_deg);
  std::string logInfo = "Gripper Pos: " + std::to_string(position_);

  // Process current sample for overcurrent protection
  if (overcurrent_protection_) {
    // Read current from the driver
    success = driver_.getCurrent(current_);
    if (!success) {
      RCLCPP_ERROR(logger_, "Failed to read current from servo");
      return hardware_interface::return_type::ERROR;
    }

    logInfo += " | Current: " + std::to_string(current_);
    overcurrent_protection_->AddCurrentSample(time, current_);
  }

  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ARServoGripperHWInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  double position_command = position_command_;

  // Apply overcurrent protection
  if (overcurrent_protection_) {
    position_command = overcurrent_protection_->AdjustGripperPosition(
        position_command, closed_position_, open_position_);
  }

  int pos_deg = linear_pos_to_servo_angle(position_command);
  std::string logInfo = "Gripper Cmd: " + std::to_string(position_command);
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());
  bool success = driver_.writePosition(pos_deg);
  if (!success) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace annin_ar4_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(annin_ar4_driver::ARServoGripperHWInterface,
                       hardware_interface::SystemInterface)
