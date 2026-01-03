#pragma once

#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "math.h"
#include "time.h"

namespace annin_ar4_driver {

class ArduinoNanoDriver {
 public:
  bool init(std::string port, int baudrate);
  void update(std::vector<double>& pos_commands,
              std::vector<double>& joint_states);
  bool getPosition(int& position);
  bool writePosition(double position);
  bool getCurrent(double& current);
  bool writeACS712Version(int version);

  ArduinoNanoDriver();

 private:
  std::string version_;
  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_port_;
  rclcpp::Logger logger_ = rclcpp::get_logger("arduino_nano_driver");

  double gripper_position_ = 0.0;
  double gripper_velocity_ = 0.0;
  double gripper_position_command_ = 0.0;

  std::string sendCommand(std::string outMsg);
  bool transmit(std::string outMsg, std::string& err);
  void receive(std::string& inMsg);

  bool checkInit(std::string msg);
};

}  // namespace annin_ar4_driver
