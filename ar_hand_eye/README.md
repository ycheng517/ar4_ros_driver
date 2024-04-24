# AR Hand Eye

Performs hand-eye calibration between a camera and the AR arm, and validates
calibration results. An Intel Realsense camera is used as an example, but any
camera can be sustituted.

The core calibration procedure uses [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2).

## Preparation

Print an aruco marker and attach it to the end of J6. Aruco marker can be printed using
[ros2_aruco](https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco?tab=readme-ov-file#generating-marker-images).

Verify that the aruco marker parameters in `config/aruco_parameters.yaml` is correct for
your marker.

Note: You may want to 3D print an extra J6 gripper mount for this purpose.

## Installation

Note: Run all commands from the repo's root directory.

Import required repos with:

```bash
vcs import . --input ar_hand_eye/hand_eye_calibration.repos
```

Install the required drivers and ROS 2 nodes for your camera. For example for Intel Realsense:

```bash
sudo apt install ros-iron-librealsense2* ros-iron-realsense2-*
```

Install dependencies of imported repos:

```bash
sudo apt update && rosdep install --from-paths . --ignore-src -y
```

## Calibrate

In one terminal, launch the hardware interface without including the gripper:

```bash
ros2 launch ar_hardware_interface ar_hardware.launch.py calibrate:=True include_gripper:=False
```

In another terminal, launch programs needed for calibration:

```bash
ros2 launch ar_hand_eye calibrate.launch.py
```

Using RViz, move the robot end effector such that the aruco marker is in view of the camera. Then, a
calibration GUI should appear on the screen. Use it to take a sample. Then move the end effector
to a few different orientations, take a sample at each orientation. When you have 5-6 samples, you
can save and exit all programs.

## Validate

In one terminal, launch the hardware interface without including the gripper:

```bash
ros2 launch ar_hardware_interface ar_hardware.launch.py calibrate:=True include_gripper:=False
```

In another terminal, launch programs needed for validation:

```bash
ros2 launch ar_hand_eye validate.launch.py
```

Move an aruco marker around the camera'a field of view. The robot arm should follow and hover above
the aruco marker.
