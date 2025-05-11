#include "annin_ar4_driver/gripper_overcurrent_protection.hpp"

#include <algorithm>

namespace annin_ar4_driver {

GripperOverCurrentProtection::GripperOverCurrentProtection(
    const rclcpp::Logger& logger, const rclcpp::Clock& clock)
    : logger_(logger), clock_(clock) {}

void GripperOverCurrentProtection::AddCurrentSample(const rclcpp::Time& time,
                                                    double current) {
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
  if (high_current_percentage > overcurrent_percent_samples_) {
    if (!overcurrent_) {
      curr_adjust_amount_ = 0.0;
      overcurrent_ = true;
    }
  }

  if (overcurrent_) {
    // If current levels are normalized, reset overcurrent flag
    if (high_current_percentage <= recovery_percent_samples_) {
      RCLCPP_INFO(
          logger_,
          "Current levels normalized: %.1f%% of measurements below threshold",
          high_current_percentage * 100.0);
      overcurrent_ = false;
    } else {
      RCLCPP_WARN_THROTTLE(
          logger_, clock_, 1000,
          "Gripper current (%.3f A) exceeded %f A for %.1f%% of time in "
          "the last %.1f seconds",
          current_, max_current_threshold_, high_current_percentage * 100.0,
          current_tracking_window_);
    }
  }
}

double GripperOverCurrentProtection::AdjustGripperPosition(
    double position_command, double min_position, double max_position) {
  if (overcurrent_) {
    curr_adjust_amount_ += overcurrent_position_increment_;
    overcurrent_cmd_pos_ = position_command;
  }

  double new_pos = position_command;
  if (overcurrent_ || position_command <= overcurrent_cmd_pos_) {
    new_pos = overcurrent_cmd_pos_ + curr_adjust_amount_;

    // Clamp new position to joint limits
    new_pos = std::clamp(new_pos, min_position, max_position);
  }

  if (new_pos != position_command) {
    RCLCPP_WARN_THROTTLE(
        logger_, clock_, 1000,
        "Adjusted gripper position from %.4f to %.4f to reduce "
        "current (now %.3f A)",
        position_command, new_pos, current_);
  }
  return new_pos;
}

}  // namespace annin_ar4_driver
