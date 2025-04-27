#include "annin_ar4_driver/gripper_thermal_protection.hpp"

#include <algorithm>

namespace annin_ar4_driver {

GripperThermalProtection::GripperThermalProtection(const rclcpp::Logger& logger,
                                                   const rclcpp::Clock& clock)
    : logger_(logger), clock_(clock) {}

void GripperThermalProtection::configureFromParams(
    const std::unordered_map<std::string, std::string>& hardware_params) {
  // Set default values

  // Override defaults with parameters if provided
  if (hardware_params.count("max_current_threshold") > 0) {
    try {
      max_current_threshold_ =
          std::stod(hardware_params.at("max_current_threshold"));
      RCLCPP_INFO(logger_, "Using max_current_threshold: %.2f A",
                  max_current_threshold_);
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "Invalid max_current_threshold parameter: %s",
                  e.what());
    }
  }

  if (hardware_params.count("high_current_percentage_threshold") > 0) {
    try {
      high_current_percentage_threshold_ =
          std::stod(hardware_params.at("high_current_percentage_threshold"));
      RCLCPP_INFO(logger_, "Using high_current_percentage_threshold: %.2f",
                  high_current_percentage_threshold_);
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_,
                  "Invalid high_current_percentage_threshold parameter: %s",
                  e.what());
    }
  }

  if (hardware_params.count("current_tracking_window") > 0) {
    try {
      current_tracking_window_ =
          std::stod(hardware_params.at("current_tracking_window"));
      RCLCPP_INFO(logger_, "Using current_tracking_window: %.2f seconds",
                  current_tracking_window_);
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "Invalid current_tracking_window parameter: %s",
                  e.what());
    }
  }

  if (hardware_params.count("adapt_position_step") > 0) {
    try {
      adapt_position_step_ =
          std::stod(hardware_params.at("adapt_position_step"));
      RCLCPP_INFO(logger_, "Using adapt_position_step: %.6f",
                  adapt_position_step_);
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "Invalid adapt_position_step parameter: %s",
                  e.what());
    }
  }
}

void GripperThermalProtection::addCurrentSample(const rclcpp::Time& time,
                                                double current) {
  if (!enabled_) {
    return;
  }

  current_ = current;

  // Add the current sample to the sliding window
  bool is_high_current = current_ > max_current_threshold_;
  current_samples_.push_back({time, is_high_current});

  // Remove samples older than the debounce window
  while (!current_samples_.empty() &&
         (time - current_samples_.front().timestamp).seconds() >
             current_tracking_window_) {
    current_samples_.pop_front();
  }

  // If there are no samples in the window, early exit
  if (current_samples_.empty()) {
    return;
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
      overheating_cmd_pos_ =
          overheating_cmd_pos_;  // Will be set by adaptGripperPosition
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
}

double GripperThermalProtection::adaptGripperPosition(double position_command,
                                                      double min_position,
                                                      double max_position) {
  if (!enabled_) {
    return position_command;
  }

  if (overheating_) {
    curr_adapt_amount_ += adapt_position_step_;
    overheating_cmd_pos_ = position_command;
  }

  double new_pos = position_command;
  if (overheating_ || position_command <= overheating_cmd_pos_) {
    new_pos = overheating_cmd_pos_ + curr_adapt_amount_;

    // Clamp new position to joint limits
    new_pos = std::clamp(new_pos, min_position, max_position);
  }

  if (new_pos != position_command) {
    RCLCPP_WARN_THROTTLE(logger_, clock_, 500,
                         "Adapted gripper position from %.4f to %.4f to reduce "
                         "current (%.3f A)",
                         position_command, new_pos, current_);
  }
  return new_pos;
}

}  // namespace annin_ar4_driver
