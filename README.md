# AR4 ROS Driver

ROS 2 driver of the AR4 robot arm from [Annin Robotics](https://www.anninrobotics.com).
Tested with ROS 2 Iron on Ubuntu 22.04. Also supports Jazzy using
[this branch](https://github.com/ycheng517/ar4_ros_driver/tree/feature/jazzy)

**Supports:**

- AR4 (Original version)
- AR4 MK2
- AR4 MK3
- AR4 servo gripper

**Features:**

- Moveit control (GUI and Python interface example)
- Gazebo simulation
- Hand-Eye calibration

This is a refresh of [ar3_core](https://github.com/ongdexter/ar3_core).

## Video Demo:

Motion Planning using RViz and Moveit:

[![AR4 ROS 2 Driver Demo](http://img.youtube.com/vi/XJCrfrW7jXE/0.jpg)](https://www.youtube.com/watch?v=XJCrfrW7jXE "AR4 ROS 2 Driver Demo")

[Startup, Calibration, and Servo Gripper Demo](https://youtu.be/PQtXFzqRtHM)

## Overview

- **ar_description**
  - Hardware description of arm & servo gripper urdf.
- **ar_hardware_interface**
  - ROS interfaces for the arm and servo gripper drivers, built on the ros2_control framework.
  - Manages joint offsets, limits and conversion between joint and actuator messages.
  - Handles communication with the microcontrollers.
- **ar_microcontrollers**
  - Firmware for the microcontrollers, ie. Teensy and Arduino Nano.
- **ar_moveit_config**
  - MoveIt module for motion planning.
  - Controlling the arm and servo gripper through Rviz.
- **ar_gazebo**
  - Simulation on Gazebo.

## Installation

- Install [ROS 2 Iron](https://docs.ros.org/en/iron/Installation.html) for Ubuntu 22.04
- Clone this repository:
  ```bash
  git clone https://github.com/ycheng517/ar4_ros_driver
  ```
- Install workspace dependencies:
  ```bash
  rosdep install --from-paths . --ignore-src -r -y
  ```
- Build the workspace:
  ```bash
  colcon build
  ```
- Source the workspace:
  ```bash
  source install/setup.bash
  ```
- Enable serial port access if you haven't already done so:
  ```bash
  sudo addgroup $USER dialout
  ```
  You will need to log out and back in for changes to take effect.

### Firmware Flashing

The Teensy and Arduino Nano sketches provided in [ar_microcontrollers](./ar_microcontrollers/)
are compatible with the default hardware. To flash it, follow the same
procedure as specified in [AR4 Robot Setup](https://www.youtube.com/watch?v=OL6lXu8VU4s).

### [Optional] Running in Docker Container

A docker container and run script has been provided that can be used to run the
robot and any GUI programs. It requires [rocker](https://github.com/osrf/rocker) to be installed. Then you can start the docker container with:

```bash
docker build -t ar4_ros_driver .
rocker --ssh --x11 --volume $(pwd):/ar4_ws/src/ar4_ros_driver -- ar4_ros_driver bash
# Feel free to adjust the volume mounting based on your project structure
```

## Usage

There are two modules that you will always need to run:

1. **Arm module** - this can be for either a real-world or simulated arm

   - For controlling the real-world arm, you will need to run the `ar_hardware_interface` module
   - For the simulated arm, you will need to run the `ar_gazebo` module
   - Either of the modules will load the necessary hardware descriptions for MoveIt

2. **MoveIt module** - the `ar_moveit_config` module provides the MoveIt interface and RViz GUI.

The various use cases of the modules and instructions to run them are described below:

---

### MoveIt Demo in RViz

If you are unfamiliar with MoveIt, it is recommended to start with this to explore planning with MoveIt in RViz. This contains neither a real-world nor a simulated arm but just a model loaded within RViz for visualisation.

The robot description, moveit interface and RViz will all be loaded in the single demo launch file

```bash
ros2 launch ar_moveit_config demo.launch.py
```

---

### Control real-world arm with MoveIt in RViz

Start the `ar_hardware_interface` module, which will load configs and the robot description:

```bash
ros2 launch ar_hardware_interface ar_hardware.launch.py \
  calibrate:=True
```

Available Launch Arguments:

- `ar_model`: The model of the AR4. Options are `mk1`, `mk2`, or `mk3`. Defaults to `mk3`.
- `calibrate`: Whether to calibrate the robot arm (determine the absolute position
  of each joint).
- `include_gripper`: Whether to include the servo gripper. Defaults to: `include_gripper:=True`.
- `serial_port`: Serial port of the Teensy board. Defaults to: `serial_port:=/dev/ttyACM0`.
- `arduino_serial_port`: Serial port of the Arduino Nano board. Defaults to `arduino_serial_port:=/dev/ttyUSB0`.

⚠️📏 Note: Calibration is required after flashing firmware to the Teensy board, and
power cycling the robot and/or the Teensy board. It can be skipped in subsequent
runs with `calibrate:=False`.

Start MoveIt and RViz:

```bash
ros2 launch ar_moveit_config ar_moveit.launch.py
```

Available Launch Arguments:

- `ar_model`: The model of the AR4. Options are `ar4` (which includes MK2) or `ar4_mk3`. Defaults to `ar4`.
- `include_gripper`: Whether to include the servo gripper. Defaults to:
  `include_gripper:=True`.
- `use_sim_time`: Make Moveit use simulation time. Should only be enabled when
  running with Gazebo. Defaults to: `use_sim_time:=False`.

You can now plan in RViz and control the real-world arm. Joint commands and joint states will be updated through the hardware interface.

---

### Control simulated arm in Gazebo with MoveIt in RViz

Start the `ar_gazebo` module, which will start the Gazebo simulator and load the robot description.

```bash
ros2 launch ar_gazebo ar_gazebo.launch.py
```

Start Moveit and RViz:

```bash
ros2 launch ar_moveit_config ar_moveit.launch.py use_sim_time:=true include_gripper:=True
```

You can now plan in RViz and control the simulated arm.

---

### Hand-Eye Calibration

See [ar4_hand_eye_calibration](https://github.com/ycheng517/ar4_hand_eye_calibration)
