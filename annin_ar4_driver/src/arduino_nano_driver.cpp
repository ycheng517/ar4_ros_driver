#include "annin_ar4_driver/arduino_nano_driver.hpp"

#define FW_VERSION "0.1.0"

namespace annin_ar4_driver {

bool ArduinoNanoDriver::init(std::string port, int baudrate) {
  // @TODO read version from config
  version_ = FW_VERSION;

  // establish connection with arduino nano board
  boost::system::error_code ec;
  serial_port_.open(port, ec);

  if (ec) {
    RCLCPP_WARN(logger_, "Failed to connect to serial port %s", port.c_str());
    return false;
  } else {
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(
        static_cast<uint32_t>(baudrate)));
    serial_port_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    RCLCPP_INFO(logger_, "Successfully connected to serial port %s",
                port.c_str());
  }

  RCLCPP_INFO(logger_, "Waiting for response from Arduino Nano on port %s",
              port.c_str());
  std::this_thread::sleep_for(std::chrono::seconds(2));
  std::string msg = "ST\n";
  std::string reply = sendCommand(msg);
  if (!checkInit(reply)) {
    return false;
  }
  RCLCPP_INFO(logger_, "Successfully initialised driver on port %s",
              port.c_str());
  return true;
}

ArduinoNanoDriver::ArduinoNanoDriver() : serial_port_(io_service_) {}

std::string ArduinoNanoDriver::sendCommand(std::string outMsg) {
  std::string errTransmit = "";
  if (!transmit(outMsg, errTransmit)) {
    RCLCPP_ERROR(logger_, "Failed to transmit message: %s",
                 errTransmit.c_str());
    return "";
  }

  std::string inMsg;
  receive(inMsg);
  return inMsg;
}

bool ArduinoNanoDriver::transmit(std::string msg, std::string& err) {
  boost::system::error_code ec;
  const auto sendBuffer = boost::asio::buffer(msg.c_str(), msg.size());

  boost::asio::write(serial_port_, sendBuffer, ec);

  if (!ec) {
    return true;
  } else {
    err = ec.message();
    return false;
  }
}

void ArduinoNanoDriver::receive(std::string& inMsg) {
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

bool ArduinoNanoDriver::checkInit(std::string msg) {
  if (msg == version_) {
    return true;
  } else {
    RCLCPP_ERROR(logger_, "Firmware version mismatch %s vs. %s", msg.c_str(),
                 version_.c_str());
    return false;
  }
}

bool ArduinoNanoDriver::getPosition(int& position) {
  std::string reply = sendCommand("SP0\n");
  if (reply == "") {
    RCLCPP_ERROR(logger_, "Failed to get position");
    return false;
  }
  position = std::stoi(reply);
  return true;
}

bool ArduinoNanoDriver::writePosition(double position) {
  std::string msg = "SV0P" + std::to_string(static_cast<int>(position)) + "\n";
  std::string reply = sendCommand(msg);
  if (reply != "Done") {
    RCLCPP_ERROR(logger_, "Failed to write position %f, got reply: %s",
                 position, reply.c_str());
    return false;
  }
  return true;
}

bool ArduinoNanoDriver::getCurrent(double& current) {
  std::string reply = sendCommand("CR\n");
  if (reply == "") {
    RCLCPP_ERROR(logger_, "Failed to get current reading");
    return false;
  }

  try {
    current = std::stod(reply);
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to convert current reading: %s", e.what());
    return false;
  }
}

bool ArduinoNanoDriver::writeACS712Version(int version) {
  std::string msg = "ACX" + std::to_string(static_cast<int>(version)) + "\n";
  std::string reply = sendCommand(msg);
  if (reply != "Done") {
    RCLCPP_ERROR(logger_, "Failed to write ACS712 Version %d, got reply: %s",
                 version, reply.c_str());
    return false;
  }
  return true;
}

}  // namespace annin_ar4_driver
