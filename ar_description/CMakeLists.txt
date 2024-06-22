cmake_minimum_required(VERSION 3.5)
project(ar_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
