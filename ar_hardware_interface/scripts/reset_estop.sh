#!/bin/bash

# This is a convenience script to reset the EStop

set -e # exit if error

if [ -z "$1" ]; then
  printf "\tUsage: ros2 run annin_ar4_driver reset_estop.sh <AR_MODEL>\n"
  printf "\twhere <AR_MODEL> is 'mk1', 'mk2' or 'mk3'\n"
  exit 1
fi

ros2 control set_hardware_component_state "$1"  active
ros2 control set_controller_state joint_trajectory_controller active
ros2 control set_controller_state joint_state_broadcaster active

exit 0
