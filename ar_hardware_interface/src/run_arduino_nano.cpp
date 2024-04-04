// main program for running the ardunio nano driver and sending some test
// commands

#include <ar_hardware_interface/arduino_nano_driver.hpp>

int main() {
  ar_hardware_interface::ArduinoNanoDriver driver;
  bool success = driver.init("/dev/ttyUSB0", 9600);
  if (!success) {
    return -1;
  }

  while (true) {
    std::string reply;
    reply = driver.writePosition(10);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    driver.writePosition(30);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  return 0;
}
