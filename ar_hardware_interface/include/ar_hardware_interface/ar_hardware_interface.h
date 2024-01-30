#pragma once

#include <boost/scoped_ptr.hpp>
#include <chrono>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "ar_hardware_driver/TeensyDriver.h"

using namespace hardware_interface;

namespace ar_hardware_interface {
class ARHardwareInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ARHardwareInterface);
  virtual ~ARHardwareInterface();

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

  void init_variables();
  // void update(const ros::TimerEvent& e);
  // void read();
  // void write(ros::Duration elapsed_time);

 private:
  rclcpp::Logger logger_ = rclcpp::get_logger("ar_hardware_interface");
  double p_error_, v_error_, e_error_;

  // Motor driver
  ar_hardware_driver::TeensyDriver driver_;
  std::vector<double> actuator_commands_;
  std::vector<double> actuator_positions_;

  // Shared memory
  int num_joints_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_offsets_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  std::vector<double> joint_position_commands_;
  std::vector<double> joint_velocity_commands_;
  std::vector<double> joint_effort_commands_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> velocity_limits_;
  std::vector<double> acceleration_limits_;

  // Misc
  double degToRad(double deg) { return deg / 180.0 * M_PI; };
  double radToDeg(double rad) { return rad / M_PI * 180.0; };
};
}  // namespace ar_hardware_interface
