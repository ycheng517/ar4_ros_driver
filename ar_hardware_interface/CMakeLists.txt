cmake_minimum_required(VERSION 3.5)
project(ar_hardware_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(ar_hardware_interface SHARED 
  src/ar_hardware_interface.cpp
  src/ar_servo_gripper_hw_interface.cpp
  src/teensy_driver.cpp
  src/arduino_nano_driver.cpp
)

target_compile_features(ar_hardware_interface PUBLIC cxx_std_17)
target_include_directories(ar_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ar_hardware_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  Boost
)

add_executable(run_arduino_nano src/run_arduino_nano.cpp)

target_link_libraries(
  run_arduino_nano
  ar_hardware_interface
)

install(TARGETS
  run_arduino_nano
  DESTINATION lib/${PROJECT_NAME}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)


install(TARGETS ar_hardware_interface
  EXPORT export_ar_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/reset_estop.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_ar_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

ament_package()
