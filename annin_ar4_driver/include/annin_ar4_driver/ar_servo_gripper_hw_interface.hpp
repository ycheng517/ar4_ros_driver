#pragma once

#include <math.h>

#include <boost/scoped_ptr.hpp>
#include <chrono>
#include <hardware_interface/system_interface.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "annin_ar4_driver/arduino_nano_driver.hpp"
#include "annin_ar4_driver/gripper_over_current_protection.hpp"

using namespace hardware_interface;

namespace annin_ar4_driver {
class ARServoGripperHWInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ARServoGripperHWInterface);

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  rclcpp::Logger logger_ = rclcpp::get_logger("ar_servo_gripper_hw_interface");
  rclcpp::Clock clock_ = rclcpp::Clock(RCL_ROS_TIME);
  int min_servo_angle_ = 55;   // closed position
  int max_servo_angle_ = 125;  // open position

  ArduinoNanoDriver driver_;
  double closed_position_;
  double open_position_;
  double position_ = 0.0;
  double velocity_ = 0.0;
  double current_ = 0.0;
  double position_command_ = 0.0;

  // Overcurrent protection (optional)
  std::unique_ptr<GripperOverCurrentProtection> overcurrent_protection_;
  bool use_overcurrent_protection_ = true;

  int linear_pos_to_servo_angle(double linear_pos) {
    double normalized_pos =
        (linear_pos - closed_position_) /
        static_cast<double>(open_position_ - closed_position_);
    return static_cast<int>(min_servo_angle_ +
                            normalized_pos *
                                (max_servo_angle_ - min_servo_angle_));
  };

  double servo_angle_to_linear_pos(int angular_pos) {
    double normalized_pos =
        (angular_pos - min_servo_angle_) /
        static_cast<double>(max_servo_angle_ - min_servo_angle_);
    return normalized_pos * (open_position_ - closed_position_) +
           closed_position_;
  };
};

}  // namespace annin_ar4_driver
