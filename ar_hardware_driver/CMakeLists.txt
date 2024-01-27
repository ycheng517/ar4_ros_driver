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

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# Declare a library
add_library(${PROJECT_NAME}
  src/TeensyDriver.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  Boost
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Export dependencies
ament_export_dependencies(rclcpp Boost)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
