#include "ar_hardware_interface/teensy_driver.hpp"

#include <chrono>
#include <thread>

#define FW_VERSION "0.0.1"

namespace ar_hardware_interface {

void TeensyDriver::init(std::string port, int baudrate, int num_joints) {
  // @TODO read version from config
  version_ = FW_VERSION;

  // establish connection with teensy board
  boost::system::error_code ec;
  serial_port_.open(port, ec);

  if (ec) {
    RCLCPP_WARN(logger_, "Failed to connect to serial port %s", port.c_str());
    return;
  } else {
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(
        static_cast<uint32_t>(baudrate)));
    serial_port_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    RCLCPP_INFO(logger_, "Successfully connected to serial port %s",
                port.c_str());
  }

  initialised_ = false;
  std::string msg = "STA" + version_ + "\n";

  while (!initialised_) {
    RCLCPP_INFO(logger_, "Waiting for response from Teensy on port %s",
                port.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    exchange(msg);
  }
  RCLCPP_INFO(logger_, "Successfully initialised driver on port %s",
              port.c_str());

  // initialise joint and encoder calibration
  num_joints_ = num_joints;
  joint_positions_deg_.resize(num_joints_);
  enc_calibrations_.resize(num_joints_);
}

TeensyDriver::TeensyDriver() : serial_port_(io_service_) {}

void TeensyDriver::setStepperSpeed(std::vector<double>& max_speed,
                                   std::vector<double>& max_accel) {
  std::string outMsg = "SS";
  for (int i = 0, charIdx = 0; i < num_joints_; ++i, charIdx += 2) {
    outMsg += 'A' + charIdx;
    outMsg += std::to_string(max_speed[i]);
    outMsg += 'A' + charIdx + 1;
    outMsg += std::to_string(max_accel[i]);
  }
  outMsg += "\n";
  exchange(outMsg);
}

// Update between hardware interface and hardware driver
void TeensyDriver::update(std::vector<double>& pos_commands,
                          std::vector<double>& joint_positions) {
  // construct update message
  std::string outMsg = "MT";
  for (int i = 0; i < num_joints_; ++i) {
    outMsg += 'A' + i;
    outMsg += std::to_string(pos_commands[i]);
  }
  outMsg += "\n";

  // run the communication with board
  exchange(outMsg);

  joint_positions = joint_positions_deg_;
}

void TeensyDriver::calibrateJoints() {
  std::string outMsg = "JC\n";
  sendCommand(outMsg);
}

void TeensyDriver::getJointPositions(std::vector<double>& joint_positions) {
  // get current joint positions
  std::string msg = "JP\n";
  exchange(msg);
  joint_positions = joint_positions_deg_;
}

// Send specific commands
void TeensyDriver::sendCommand(std::string outMsg) { exchange(outMsg); }

// Send msg to board and collect data
void TeensyDriver::exchange(std::string outMsg) {
  std::string inMsg;
  std::string errTransmit = "";

  if (!transmit(outMsg, errTransmit)) {
    RCLCPP_ERROR(logger_, "Error in transmit: %s", errTransmit.c_str());
  }

  bool done = false;
  while (!done) {
    receive(inMsg);
    // parse msg
    std::string header = inMsg.substr(0, 2);
    if (header == "ST") {
      // init acknowledgement
      checkInit(inMsg);
      done = true;
    } else if (header == "JC") {
      // encoder calibration values
      updateEncoderCalibrations(inMsg);
      done = true;
    } else if (header == "JP") {
      // encoder steps
      updateJointPositions(inMsg);
      done = true;
    } else if (header == "DB") {
      // debug message
      RCLCPP_DEBUG(logger_, "Debug message: %s", inMsg.c_str());
      done = true;
    } else {
      // unknown header
      RCLCPP_WARN(logger_, "Unknown header %s", header.c_str());
      done = true;
    }
  }
}

bool TeensyDriver::transmit(std::string msg, std::string& err) {
  boost::system::error_code ec;
  const auto sendBuffer = boost::asio::buffer(msg.c_str(), msg.size());

  boost::asio::write(serial_port_, sendBuffer, ec);

  if (!ec) {
    return true;
  } else {
    err = "Error in transmit";
    return false;
  }
}

void TeensyDriver::receive(std::string& inMsg) {
  char c;
  std::string msg = "";
  bool eol = false;
  while (!eol) {
    boost::asio::read(serial_port_, boost::asio::buffer(&c, 1));
    switch (c) {
      case '\r':
        break;
      case '\n':
        eol = true;
        break;
      default:
        msg += c;
    }
  }
  inMsg = msg;
}

void TeensyDriver::checkInit(std::string msg) {
  std::size_t ack_idx = msg.find("A", 2) + 1;
  std::size_t version_idx = msg.find("B", 2) + 1;
  int ack = std::stoi(msg.substr(ack_idx, version_idx));
  if (ack) {
    initialised_ = true;
  } else {
    std::string version = msg.substr(version_idx);
    RCLCPP_ERROR(logger_, "Firmware version mismatch %s", version.c_str());
  }
}

void TeensyDriver::updateEncoderCalibrations(std::string msg) {
  size_t idx1 = msg.find("A", 2) + 1;
  size_t idx2 = msg.find("B", 2) + 1;
  size_t idx3 = msg.find("C", 2) + 1;
  size_t idx4 = msg.find("D", 2) + 1;
  size_t idx5 = msg.find("E", 2) + 1;
  size_t idx6 = msg.find("F", 2) + 1;
  enc_calibrations_[0] = std::stoi(msg.substr(idx1, idx2 - idx1));
  enc_calibrations_[1] = std::stoi(msg.substr(idx2, idx3 - idx2));
  enc_calibrations_[2] = std::stoi(msg.substr(idx3, idx4 - idx3));
  enc_calibrations_[3] = std::stoi(msg.substr(idx4, idx5 - idx4));
  enc_calibrations_[4] = std::stoi(msg.substr(idx5, idx6 - idx5));
  enc_calibrations_[5] = std::stoi(msg.substr(idx6));

  // @TODO update config file
  RCLCPP_INFO(logger_, "Successfully updated encoder calibrations");
}

void TeensyDriver::updateJointPositions(std::string msg) {
  size_t idx1 = msg.find("A", 2) + 1;
  size_t idx2 = msg.find("B", 2) + 1;
  size_t idx3 = msg.find("C", 2) + 1;
  size_t idx4 = msg.find("D", 2) + 1;
  size_t idx5 = msg.find("E", 2) + 1;
  size_t idx6 = msg.find("F", 2) + 1;
  joint_positions_deg_[0] = std::stod(msg.substr(idx1, idx2 - idx1));
  joint_positions_deg_[1] = std::stod(msg.substr(idx2, idx3 - idx2));
  joint_positions_deg_[2] = std::stod(msg.substr(idx3, idx4 - idx3));
  joint_positions_deg_[3] = std::stod(msg.substr(idx4, idx5 - idx4));
  joint_positions_deg_[4] = std::stod(msg.substr(idx5, idx6 - idx5));
  joint_positions_deg_[5] = std::stod(msg.substr(idx6));
}

}  // namespace ar_hardware_interface
