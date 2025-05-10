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
  double AdaptGripperPosition(double position_command, double min_position,
                              double max_position);

 private:
  // Current monitoring
  struct CurrentSample {
    rclcpp::Time timestamp;
    bool is_high_current;
  };

  std::deque<CurrentSample> current_samples_;
  bool overcurrent_ = false;
  double current_tracking_window_ = 2.0;  // seconds window for monitoring
  double max_current_threshold_ = 2.0;    // Amps
  double high_current_percentage_threshold_ = 0.75;

  // Adaptive current limiting
  double overcurrent_position_increment_ = 0.0001;  // Position increment during overcurrent recovery
  double curr_adapt_amount_ = 0.0;       // Amount to adapt position
  double overcurrent_cmd_pos_ = 0.0;     // Commanded position when overcurrent

  // Current monitoring state
  double current_ = 0.0;

  // Logging
  rclcpp::Logger logger_;
  rclcpp::Clock clock_;
};

}  // namespace annin_ar4_driver
