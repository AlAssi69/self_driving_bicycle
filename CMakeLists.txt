cmake_minimum_required(VERSION 3.8)
project(self_driving_bicycle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)

add_executable(odom_broadcaster src/odom_broadcaster.cpp)

ament_target_dependencies(
  odom_broadcaster
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  tf2_msgs
)

install(
  TARGETS odom_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config include launch maps src urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()