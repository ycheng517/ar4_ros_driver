# ar_hardware_driver

This module contains the hardware drivers for the AR4 robot.

## Teensy Driver

This driver serves as a bridge between the higher level software handled by
the hardware_interface and the Teensy. Since the original design only has a
single motor controller in the form of the Teensy, which then manages the
individual stepper drivers, I've decided to keep all communication
synchronous to avoid complications with thread safety.
