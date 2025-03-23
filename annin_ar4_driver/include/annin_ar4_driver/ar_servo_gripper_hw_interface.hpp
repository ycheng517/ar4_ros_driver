#pragma once

#include <math.h>

#include <boost/scoped_ptr.hpp>
#include <chrono>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "annin_ar4_driver/arduino_nano_driver.hpp"

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
  double servo_arm_length_ = 0.023;  // meters
  // offset in degrees for the zero position, in case the servo can't reach
  // 0 degrees due to mechanical tolerance issues.
  double zero_deg_offset_ = 2;

  ArduinoNanoDriver driver_;
  double position_ = 0.0;
  double velocity_ = 0.0;
  double position_command_ = 0.0;

  int linear_to_angular_pos(double linear_pos) {
    return static_cast<int>(asin(linear_pos / servo_arm_length_) * 180 / M_PI +
                            zero_deg_offset_);
  };

  double angular_to_linear_pos(int angular_pos) {
    return servo_arm_length_ *
           sin((angular_pos - zero_deg_offset_) * M_PI / 180);
  };
};
}  // namespace annin_ar4_driver
