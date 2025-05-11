#pragma once

#include <deque>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

namespace annin_ar4_driver {

class GripperOverCurrentProtection {
 public:
  GripperOverCurrentProtection(const rclcpp::Logger& logger,
                               const rclcpp::Clock& clock);

  void AddCurrentSample(const rclcpp::Time& time, double current);
  double AdjustGripperPosition(double position_command, double min_position,
                               double max_position);

 private:
  // For current measurement
  struct CurrentSample {
    rclcpp::Time timestamp;
    bool is_high_current;
  };

  std::deque<CurrentSample> current_samples_;
  bool overcurrent_ = false;
  double current_ = 0.0;
  // Total amount that the commanded position has been adjusted
  double curr_adjust_amount_ = 0.0;
  // The original commanded position when overcurrent started
  double overcurrent_cmd_pos_ = 0.0;

  rclcpp::Logger logger_;
  rclcpp::Clock clock_;

  // Parameters for tuning overcurrent protection
  double current_tracking_window_ = 2.0;  // seconds window for monitoring
  double max_current_threshold_ = 2.0;    // Amps
  // Percentage of samples that must be over the max current threshold to
  // trigger overcurrent
  double overcurrent_percent_samples_ = 0.75;
  // Percentage of samples that must be under the max current threshold to
  // trigger recovery
  double recovery_percent_samples_ = 0.6;
  // Amount to adjust position by per cycle while overcurrent
  double overcurrent_position_increment_ = 0.00002;
};

}  // namespace annin_ar4_driver
