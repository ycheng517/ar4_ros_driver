#pragma once

#include <deque>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

namespace annin_ar4_driver {

class GripperThermalProtection {
 public:
  GripperThermalProtection(const rclcpp::Logger& logger,
                           const rclcpp::Clock& clock);

  // New method to configure from hardware parameters
  void configureFromParams(
      const std::unordered_map<std::string, std::string>& hardware_params);

  void addCurrentSample(const rclcpp::Time& time, double current);
  double adaptGripperPosition(double position_command, double min_position,
                              double max_position);
  bool isEnabled() const { return enabled_; }
  void setEnabled(bool enabled) { enabled_ = enabled; }

 private:
  // Current monitoring
  struct CurrentSample {
    rclcpp::Time timestamp;
    bool is_high_current;
  };

  std::deque<CurrentSample> current_samples_;
  bool overheating_ = false;
  double current_tracking_window_ = 2.0;  // seconds window for monitoring
  double max_current_threshold_ = 2.0;    // Amps
  double high_current_percentage_threshold_ = 0.75;

  // Adaptive current limiting
  double adapt_position_step_ = 0.0001;  // Step size for gradual movement
  double curr_adapt_amount_ = 0.0;       // Amount to adapt position
  double overheating_cmd_pos_ = 0.0;     // Commanded position when overheating

  // Current monitoring state
  double current_ = 0.0;
  bool enabled_ = true;

  // Logging
  rclcpp::Logger logger_;
  rclcpp::Clock clock_;
};

}  // namespace annin_ar4_driver
