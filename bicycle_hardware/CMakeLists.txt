cmake_minimum_required(VERSION 3.16)
project(bicycle_hardware LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Threads REQUIRED)

# The protocol and firmware headers of the Arduino sketch are shared with the host
# (#include "bicycle_uno/protocol.hpp").
set(SHARED_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/firmware
)

# Protocol, models, configuration and serial port, used by the plugin and the tests.
add_library(bicycle_hardware_core STATIC
  src/drive_model.cpp
  src/hardware_config.cpp
  src/protocol.cpp
  src/serial_port.cpp
)
set_target_properties(bicycle_hardware_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(bicycle_hardware_core PUBLIC ${SHARED_INCLUDE_DIRS})
target_link_libraries(bicycle_hardware_core PUBLIC hardware_interface::hardware_interface)

# The ros2_control plugin. No C++ API is exported: only the plugin library is installed.
add_library(bicycle_hardware SHARED src/bicycle_system.cpp)
target_link_libraries(bicycle_hardware PRIVATE
  bicycle_hardware_core
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(hardware_interface bicycle_hardware.xml)

# Simulated Arduino on a pseudo-terminal: the tests' stand-in and a dry-run tool.
add_library(bicycle_fake_mcu STATIC src/fake_mcu.cpp)
target_include_directories(bicycle_fake_mcu PUBLIC ${SHARED_INCLUDE_DIRS})
target_link_libraries(bicycle_fake_mcu PUBLIC Threads::Threads)
add_executable(fake_mcu tools/fake_mcu_main.cpp)
target_link_libraries(fake_mcu PRIVATE bicycle_fake_mcu)

install(TARGETS bicycle_hardware LIBRARY DESTINATION lib)
install(TARGETS fake_mcu RUNTIME DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY config firmware launch udev
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_firmware_core test/test_firmware_core.cpp)
  target_include_directories(test_firmware_core PRIVATE firmware)

  foreach(test_name test_drive_model test_hardware_config test_protocol)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} bicycle_hardware_core)
  endforeach()

  # Loads the installed plugin through pluginlib and drives it against the fake Arduino.
  ament_add_gtest(test_bicycle_system test/test_bicycle_system.cpp TIMEOUT 120)
  target_link_libraries(test_bicycle_system
    bicycle_fake_mcu
    hardware_interface::hardware_interface
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
  )
endif()

ament_package()
