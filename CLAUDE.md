# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

AR4 ROS Driver is a ROS 2 driver for the AR4 robot arm from Annin Robotics. The project consists of four main packages:

- **annin_ar4_description**: Hardware description (URDF) files for the arm and servo gripper
- **annin_ar4_driver**: ROS2 control hardware interface, manages joint offsets/limits and microcontroller communication
- **annin_ar4_firmware**: Arduino/Teensy firmware for the robot controllers
- **annin_ar4_moveit_config**: MoveIt configuration for motion planning and RViz control
- **annin_ar4_gazebo**: Gazebo simulation support

## Common Commands

### Building the Workspace
```bash
# Install dependencies
rosdep install --from-paths . --ignore-src -r -y

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

### Running the System

#### Real Robot Control
```bash
# Start the driver (with calibration)
ros2 launch annin_ar4_driver driver.launch.py calibrate:=True

# Start MoveIt and RViz
ros2 launch annin_ar4_moveit_config moveit.launch.py
```

#### Simulation
```bash
# Start Gazebo simulation
ros2 launch annin_ar4_gazebo gazebo.launch.py

# Start MoveIt with simulation time
ros2 launch annin_ar4_moveit_config moveit.launch.py use_sim_time:=true include_gripper:=True
```

#### Demo Mode (RViz only)
```bash
ros2 launch annin_ar4_moveit_config demo.launch.py
```

### Emergency Stop Reset
```bash
ros2 run annin_ar4_driver reset_estop.sh <AR_MODEL>
# where AR_MODEL is mk1, mk2, or mk3
```

## Key Configuration Files

- `annin_ar4_driver/config/driver.yaml`: Calibration sequence and velocity control settings
- `annin_ar4_driver/config/controllers.yaml`: ros2_control controller configuration
- `annin_ar4_driver/config/joint_offsets/`: Joint offset calibration files for each model (mk1/mk2/mk3)
- `annin_ar4_driver/config/gripper_driver.yaml`: Servo gripper configuration
- `annin_ar4_moveit_config/config/`: MoveIt planning and servo configuration

## Architecture Notes

### Hardware Interface
The system uses ros2_control framework with custom hardware interfaces:
- `ar_hardware_interface.cpp`: Main arm controller interface
- `ar_servo_gripper_hw_interface.cpp`: Gripper controller interface
- Communication with Teensy (arm) via `/dev/ttyACM0` and Arduino Nano (gripper) via `/dev/ttyUSB0`

### Control Modes
- Default: Velocity-based joint trajectory control (faster, smoother)
- Alternative: Position-only control (set `velocity_control_enabled: false` in driver.yaml)

### Calibration System
- Supports multiple calibration sequences (configured via `calib_sequence` in driver.yaml)
- Type 0: All joints simultaneously
- Type 1-2: Grouped calibration
- Type 3: Individual joint calibration (current default: '3543210')

### Supported Models
- AR4 MK1, MK2, MK3 with different joint limits and offsets
- Optional servo gripper with overcurrent protection

## Launch Arguments

### Driver Launch
- `ar_model`: mk1/mk2/mk3 (default: mk3)
- `calibrate`: True/False (default: True)
- `include_gripper`: True/False (default: True)
- `serial_port`: Teensy port (default: /dev/ttyACM0)
- `arduino_serial_port`: Arduino port (default: /dev/ttyUSB0)

### MoveIt Launch
- `ar_model`: mk1/mk2/mk3 (default: mk3)
- `include_gripper`: True/False (default: True)
- `use_sim_time`: True/False (default: False, set True for Gazebo)

## Development Notes

- Built with ROS 2 Jazzy on Ubuntu 24.04
- Uses ament_cmake build system
- Hardware interface plugins exported via pluginlib
- Firmware requires Bounce2 library for Arduino IDE
- Docker support available with rocker for GUI applications