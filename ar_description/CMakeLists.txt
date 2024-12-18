cmake_minimum_required(VERSION 3.5)
project(annin_ar4_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
