cmake_minimum_required(VERSION 3.5)
project(ar_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${interactive_markers_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_visual_tools_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${moveit_ros_perception_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${geometric_shapes_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# # Executable
# add_executable(move_group_demo src/move_group_demo.cpp)
# ament_target_dependencies(move_group_demo
#   rclcpp
#   interactive_markers
#   moveit_core
#   moveit_visual_tools
#   moveit_ros_planning
#   moveit_ros_planning_interface
#   moveit_ros_perception
#   pluginlib
#   geometric_shapes
#   tf2_ros
#   tf2_eigen
#   tf2_geometry_msgs
#   Eigen3
#   Boost
# )
# 
# # Installation
# install(TARGETS move_group_demo
#   DESTINATION lib/${PROJECT_NAME}
# )
# 
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
