cmake_minimum_required(VERSION 3.5)
project(ar_hardware_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories(include)

# Declare a library
add_library(ar_hardware_driver
  src/TeensyDriver.cpp
)

ament_target_dependencies(ar_hardware_driver
  rclcpp
  Boost
)

# Install targets
install(TARGETS ar_hardware_driver
  EXPORT ar_hardware_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Export dependencies
ament_export_dependencies(rclcpp Boost)
ament_export_include_directories(include)
ament_export_libraries(ar_hardware_driver)

ament_package()
