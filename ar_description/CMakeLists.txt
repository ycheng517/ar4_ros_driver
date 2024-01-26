cmake_minimum_required(VERSION 3.5)
project(ar_description)

find_package(ament_cmake REQUIRED)

catkin_package()

find_package(roslaunch)

install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
