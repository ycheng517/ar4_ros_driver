cmake_minimum_required(VERSION 2.8.3)
project(ar_hardware_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ar_hardware_driver
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(ar_hardware_interface SHARED src/ar_hardware_interface.cpp)
target_compile_features(ar_hardware_interface PUBLIC cxx_std_17)
target_include_directories(ar_hardware_interface PUBLIC include)

ament_target_dependencies(ar_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_example_1.xml)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install targets
install(TARGETS ar_hardware_interface
  EXPORT ar_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(ar_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
