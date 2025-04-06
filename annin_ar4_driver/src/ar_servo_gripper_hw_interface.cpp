#include <algorithm>
#include <annin_ar4_driver/ar_servo_gripper_hw_interface.hpp>
#include <sstream>

namespace annin_ar4_driver {

hardware_interface::CallbackReturn ARServoGripperHWInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
  RCLCPP_INFO(logger_, "Initializing hardware interface...");

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;

  // Extract position limits from gripper_jaw1_joint info
  RCLCPP_INFO(logger_, "Extracting joint limits...");
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

  std::string serial_port = info_.hardware_parameters.at("serial_port");
  int baud_rate = 9600;
  bool success = driver_.init(serial_port, baud_rate);
  if (!success) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ARServoGripperHWInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Activating hardware interface...");

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

  // Read current from the driver
  success = driver_.getCurrent(current_);
  if (!success) {
    RCLCPP_ERROR(logger_, "Failed to read current from servo");
    return hardware_interface::return_type::ERROR;
  }

  std::string logInfo = "Gripper Pos: " + std::to_string(position_) +
                        ", Current: " + std::to_string(current_);
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());

  // Add the current sample to the sliding window
  bool is_high_current = current_ > max_current_threshold_;
  current_samples_.push_back({time, is_high_current});

  // Remove samples older than the debounce window
  while (!current_samples_.empty() &&
         (time - current_samples_.front().timestamp).seconds() >
             current_tracking_window_) {
    current_samples_.pop_front();
  }

  // If there are no samples in the window, early exit with OK status
  if (current_samples_.empty()) {
    return hardware_interface::return_type::OK;
  }

  // Calculate the percentage of high current samples in the window
  int high_current_count = 0;
  for (const auto& sample : current_samples_) {
    if (sample.is_high_current) {
      high_current_count++;
    }
  }

  double high_current_percentage =
      static_cast<double>(high_current_count) / current_samples_.size();

  // Check if high current percentage exceeds threshold
  if (high_current_percentage > high_current_percentage_threshold_) {
    if (!overheating_) {
      RCLCPP_WARN(logger_,
                  "Servo current exceeded threshold for %.1f%% of time in "
                  "%.1f second window",
                  high_current_percentage * 100.0, current_tracking_window_);
      overheating_ = true;
      curr_adapt_amount_ = 0.0;
      overheating_cmd_pos_ = position_command_;
    }
  }
  // If current levels are normalized, reset overheating flag
  else if (overheating_ && high_current_percentage <=
                               high_current_percentage_threshold_ * 0.8) {
    RCLCPP_INFO(
        logger_,
        "Current levels normalized: %.1f%% of measurements below threshold",
        high_current_percentage * 100.0);
    overheating_ = false;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ARServoGripperHWInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  double position_command = position_command_;

  // Adaptively modify position to reduce current if overheating
  position_command = adaptGripperPosition(position_command);

  int pos_deg = linear_pos_to_servo_angle(position_command);
  std::string logInfo = "Gripper Cmd: " + std::to_string(position_command);
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());
  bool success = driver_.writePosition(pos_deg);
  if (!success) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

double ARServoGripperHWInterface::adaptGripperPosition(
    double position_command) {
  if (overheating_) {
    curr_adapt_amount_ += adapt_position_step_;
  }

  double new_pos = position_command;
  if (overheating_ || position_command <= overheating_cmd_pos_) {
    new_pos = overheating_cmd_pos_ + curr_adapt_amount_;

    // Clamp new position to joint limits
    new_pos = std::clamp(new_pos, closed_position_, open_position_);
  }

  if (new_pos != position_command) {
    RCLCPP_WARN_THROTTLE(logger_, clock_, 500,
                         "Adapted gripper position from %.4f to %.4f to reduce "
                         "current ( %.3f A)",
                         position_command, new_pos, current_);
  }
  return new_pos;
}

}  // namespace annin_ar4_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(annin_ar4_driver::ARServoGripperHWInterface,
                       hardware_interface::SystemInterface)
