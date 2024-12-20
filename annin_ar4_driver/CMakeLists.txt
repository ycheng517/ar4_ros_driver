cmake_minimum_required(VERSION 3.5)
project(annin_ar4_driver)

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

add_library(annin_ar4_driver SHARED 
  src/ar_hardware_interface.cpp
  src/ar_servo_gripper_hw_interface.cpp
  src/teensy_driver.cpp
  src/arduino_nano_driver.cpp
)

target_compile_features(annin_ar4_driver PUBLIC cxx_std_17)
target_include_directories(annin_ar4_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(annin_ar4_driver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  Boost
)

add_executable(run_arduino_nano src/run_arduino_nano.cpp)

target_link_libraries(
  run_arduino_nano
  annin_ar4_driver
)

install(TARGETS
  run_arduino_nano
  DESTINATION lib/${PROJECT_NAME}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)


install(TARGETS annin_ar4_driver
  EXPORT export_annin_ar4_driver
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

ament_export_targets(export_annin_ar4_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

ament_package()
