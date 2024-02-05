# AR ROS Driver

ROS 2 driver of the AR4 robot arm from Annin Robotics. Tested with ROS 2 Iron on Ubuntu 22.04. This is a refresh of [ar3_core](https://github.com/ongdexter/ar3_core).

- [Overview](#Overview)
- [Installation](#Installation)
- [Usage](#Usage)

[![](http://img.youtube.com/vi/6D4vdhJlLsQ/0.jpg)](http://www.youtube.com/watch?v=6D4vdhJlLsQ "AR3 with ROS and MoveIt")

## Overview

- **ar_description**
  - Hardware description of arm, urdf etc.
- **ar_hardware_interface**
  - ROS interface for the hardware driver, built on the ros2_control framework
  - Manages joint offsets, limits and conversion between joint and actuator messages
- **ar_hardware_driver**
  - Handles communication with the motor controllers
- **ar_microcontrollers**
  - Firmware for the motor controller ie. Teensy
- **ar_moveit_config**
  - MoveIt module for motion planning
  - Controlling the arm through Rviz
- **ar_control** (Work in progress)
  - Controlling the arm through the MoveIt user interfaces
  - Provides demo for the move group interface
- **ar_gazebo** (Work in progress)
  - Simulation on Gazebo

## Installation

- Install [ROS 2 Iron](https://docs.ros.org/en/iron/Installation.html) and [MoveIt 2](https://moveit.ros.org/install-moveit2/binary/) for Ubuntu 22.04
- Clone this repository:
  ```
  git clone https://github.com/ycheng517/ar_ros_driver
  ```
- Install workspace dependencies:
  ```
  rosdep install --from-paths src --ignore-src -r -y
  ```
- Build the workspace:
  ```
  colcon build
  ```
- Source the workspace:
  ```
  source install/setup.bash
  ```
- Enable serial port access if you haven't already done so:
  ```
  sudo addgroup $USER dialout
  ```
  You will need to log out and back in for changes to take effect.

## Setup

- **Teensy Sketch**
  - Both teensy sketches provided are compatible with the default hardware. Refer to the module for more information.

## Usage

There are two modules that you will always need to run:

1. **Arm module** - this can be for either a real-world or simulated arm

   - For controlling the real-world arm, you will need to run the `ar_hardware_interface` module
   - For the simulated arm, you will need to run the `ar_gazebo` module
   - Either of the modules will load the necessary hardware descriptions for MoveIt

2. **MoveIt module** - the `ar_moveit_config` module provides the MoveIt interface and RViz GUI, and the `ar_control` module provides the MoveIt user interface for programmatically setting goals

The various use cases of the modules and instructions to run them are described below:

---

### MoveIt Demo in RViz

If you are unfamiliar with MoveIt, it is recommended to start with this to explore planning with MoveIt in RViz. This contains neither a real-world nor a simulated arm but just a model loaded within RViz for visualisation.

- The robot description, moveit interface and RViz will all be loaded in the single demo launch file
  ```
  roslaunch ar_moveit_config demo.launch
  ```

---

### Control real-world arm with MoveIt in RViz

- Start the `ar_hardware_interface` module, which will load configs and the robot description

  ```bash
  roslaunch ar_hardware_interface ar3_hardware_bringup.launch \
    serial_port:=/dev/ttyACM0 \
    calibrate:=True
  ```

- Start MoveIt and RViz

  ```bash
  roslaunch ar_moveit_config ar3_moveit_bringup.launch
  ```

  You can now plan in RViz and control the real-world arm. Joint commands and joint states will be updated through the hardware interface.

---

### Work in Progress: Control simulated arm in Gazebo with MoveIt in RViz

- Start the `ar_gazebo` module, which will start the Gazebo simulator and load the robot description
  ```
  roslaunch ar_gazebo ar_gazebo_bringup.launch
  ```
- Start Moveit and RViz
  ```
  roslaunch ar_moveit_config ar3_moveit_bringup.launch
  ```
  You can now plan in RViz and control the simulated arm.

---

### Work in Progress: Control arm with Move Group Interface

**It is recommended to run this demo with the simulated arm first to make sure that the programmed goals are safe for your environment (and your arm). Needless to say, the same applies when programming your own tasks.**

This is a demo modified from the official MoveIt tutorials. As opposed to manually setting goals through RViz, the move group interface allows us to programmatically plan and execute movements, and provides more functionality such as specifying path constraints and planning Cartesian movements. This also enables much more complex tasks, planning around obstacles etc.

- Start the `ar_gazebo` module, which will start the Gazebo simulator and load the robot description  
  _For controlling the real-world arm, you will just need to run `ar_hardware_interface` instead of `ar_gazebo` as described above._
  ```
  roslaunch ar_gazebo ar_gazebo_bringup.launch
  ```
- Start Moveit and RViz
  ```
  roslaunch ar_moveit_config ar3_moveit_bringup.launch
  ```
- Start the move group demo
  ```
  roslaunch ar_control move_group_demo.launch
  ```
  Follow the command-line instructions to step through the demo. See `ar_control` for more details.

---
