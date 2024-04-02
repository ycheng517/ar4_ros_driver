// main program for running the ardunio nano driver and sending some test
// commands

#include <ar_hardware_interface/arduino_nano_driver.hpp>

int main() {
  ar_gripper_hardware_interface::ArduinoNanoDriver driver;
  bool success = driver.init("/dev/ttyUSB0", 115200);
  if (!success) {
    return -1;
  }

  while (true) {
    std::string reply;
    reply = driver.sendCommand("SV0P10\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    driver.sendCommand("SV0P20\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  return 0;
}
