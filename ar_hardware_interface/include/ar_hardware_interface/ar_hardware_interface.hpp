#pragma once

#include <boost/scoped_ptr.hpp>
#include <chrono>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "ar_hardware_interface/teensy_driver.hpp"

using namespace hardware_interface;

namespace ar_hardware_interface {
class ARHardwareInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ARHardwareInterface);

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
  rclcpp::Logger logger_ = rclcpp::get_logger("ar_hardware_interface");
  rclcpp::Clock clock_ = rclcpp::Clock(RCL_ROS_TIME);

  // Motor driver
  TeensyDriver driver_;
  std::vector<double> actuator_pos_commands_;
  std::vector<double> actuator_vel_commands_;
  std::vector<double> actuator_positions_;
  std::vector<double> actuator_velocities_;

  // Shared memory
  std::vector<double> joint_offsets_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  std::vector<double> joint_position_commands_;
  std::vector<double> joint_velocity_commands_;
  std::vector<double> joint_effort_commands_;

  // Misc
  void init_variables();
  double degToRad(double deg) { return deg / 180.0 * M_PI; };
  double radToDeg(double rad) { return rad / M_PI * 180.0; };
};
}  // namespace ar_hardware_interface
