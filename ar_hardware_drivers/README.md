# ar3_hardware_drivers
This module contains the hardware drivers for the AR3. 

## Teensy Driver
This driver serves as a bridge between the higher level software handled by the hardware_interface and the Teensy. Since the original AR3 design only has a single motor controller in the form of the Teensy, which then manages the individual stepper drivers, I've decided to keep all communication synchronous to avoid complications with thread safety. In an effort to keep this work accessible to the general user, the message protocol also follows the standard used for ARCS.

The config file `hardware_driver.yaml` can be found in the hardware_interface configs where it is loaded to the parameter server. You will need to define the port and baudrate settings for the Teensy in that config file. More details are provided in `ar3_hardware_interface`.